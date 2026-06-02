// SPDX-License-Identifier: GPL-2.0-only
/*
 * Cypress CY8CTMA395 Touchscreen Input Driver (serdev)
 *
 * This is a kernel serdev touchscreen driver for the Cypress CY8CTMA395
 * touchscreen controller as used in the HP TouchPad.
 *
 * The touchscreen sends capacitance data over UART at 4 Mbps.
 * I2C is used for initialization and power management.
 *
 * Based on userspace ts_srv by:
 *   jonpry @ gmail - math and device output understanding
 *   Oleg Drokin <green@linuxhacker.ru> - uinput implementation
 *   Rafael Brune <mail@rbrune.de> - multitouch detection
 *   Dees_Troy - various improvements
 *   Jordan Patterson (jyxent) - tracking ID code
 *
 * Copyright (c) 2011 CyanogenMod Touchpad Project
 * Copyright (c) 2024 Linux kernel adaptation
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/serdev.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/slab.h>

#define CY8CTMA395_TS_NAME	"cy8ctma395-ts"

/* Touchscreen matrix dimensions */
#define X_AXIS_POINTS		30
#define Y_AXIS_POINTS		40
#define MATRIX_SIZE		(X_AXIS_POINTS * Y_AXIS_POINTS)

/* Screen resolution */
#define X_RESOLUTION		1024
#define Y_RESOLUTION		768

/* Touch detection thresholds */
#define TOUCH_INITIAL_THRESH	32
#define TOUCH_CONTINUE_THRESH	26
#define TOUCH_DELAY_THRESH	28
#define TOUCH_DELAY_COUNT	5
#define LARGE_AREA_UNPRESS	22
#define LARGE_AREA_FRINGE	5

/* Tracking limits */
#define MAX_TOUCH		10
#define MAX_DELTA_SQ		(130 * 130)
#define TRACKING_ID_MAX		2147483000

/* Protocol constants */
#define RECV_BUF_SIZE		1540
#define FRAME_START		0xFF
#define FRAME_ROW_DATA		0x43
#define FRAME_SCAN_COMPLETE	0x47
#define ROW_DATA_SIZE		44

/* I2C address */
#define CY8CTMA395_I2C_ADDR	0x67

/* Debounce settings */
#define HOVER_DEBOUNCE_RADIUS	2
#define HOVER_DEBOUNCE_DELAY	30
#define PIXELS_PER_POINT	25

/*
 * Hard cap on flood-fill recursion depth. Touch regions are bounded by
 * the matrix (30 x 40 = 1200 cells), but the recursive cy8ctma395_ts_*
 * helpers below take ~96 bytes of stack each, so a worst-case 1200-deep
 * call exceeds the 8-16 KiB kernel stack. Realistic finger contacts span
 * ~5-30 cells; a palm spans ~50; anything past this cap is pathological
 * UART data we should abandon rather than walk into a stack overflow.
 */
#define CY8CTMA395_FLOOD_DEPTH_MAX	64

struct cy8ctma395_touchpoint {
	int pw;			/* pressure/weight */
	int i, j;		/* position in matrix */
	int x, y;		/* screen coordinates */
	int unfiltered_x, unfiltered_y;
	int tracking_id;
	int prev_loc;
	int touch_major;
	int highest_val;
	int touch_delay;
	int hover_x, hover_y;
	int hover_delay;
	int distance;
	int input_slot;		/* allocated MT slot, -1 if none */
};

struct cy8ctma395_ts_data {
	struct serdev_device *serdev;
	struct i2c_client *i2c;
	struct input_dev *input;

	/* GPIOs */
	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_wake;

	/* Power */
	struct regulator *vdd;

	/* Protocol parser state */
	u8 cline[64];
	unsigned int cidx;
	unsigned int rows_received;

	/* Capacitance matrix */
	u8 matrix[X_AXIS_POINTS][Y_AXIS_POINTS];
	int invalid_matrix[X_AXIS_POINTS][Y_AXIS_POINTS];

	/* Touch tracking - 3 frames for filtering */
	struct cy8ctma395_touchpoint tp[3][MAX_TOUCH];
	int tpoint, prevtpoint, prev2tpoint;
	int previoustpc;
	s32 next_tracking_id;

	/* Slot tracking */
	int slot_in_use[MAX_TOUCH];

	/* Thresholds (configurable) */
	int touch_initial_thresh;
	int touch_continue_thresh;
	int touch_delay_thresh;
	int touch_delay_count;

	bool powered;

	/*
	 * Serialises the parser/matrix state between the serdev RX callback
	 * (cy8ctma395_ts_receive_buf -> process_data) and the suspend / resume
	 * / remove paths (liftoff / clear_arrays / power on/off). Without this
	 * a 4 Mbps UART chunk can land mid-suspend, race with liftoff() walking
	 * tp[][] and corrupt the slot bookkeeping so the next session reports
	 * stale tracking IDs.
	 */
	struct mutex state_lock;
};

/* Approximate pow(x, 1.5) using integer math: x * sqrt(x) */
static int pow_1_5(int val)
{
	int x, x1;

	if (val <= 0)
		return 0;

	/* Integer square root via Newton-Raphson */
	x = val;
	x1 = (x + 1) / 2;
	while (x1 < x) {
		x = x1;
		x1 = (x + val / x) / 2;
	}

	return val * x;
}

static void cy8ctma395_ts_clear_arrays(struct cy8ctma395_ts_data *ts)
{
	int i, j;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < MAX_TOUCH; j++) {
			ts->tp[i][j].pw = -1000;
			ts->tp[i][j].i = -1000;
			ts->tp[i][j].j = -1000;
			ts->tp[i][j].tracking_id = -1;
			ts->tp[i][j].prev_loc = -1;
			ts->tp[i][j].distance = 0;
			ts->tp[i][j].touch_major = 0;
			ts->tp[i][j].x = -1000;
			ts->tp[i][j].y = -1000;
			ts->tp[i][j].unfiltered_x = -1000;
			ts->tp[i][j].unfiltered_y = -1000;
			ts->tp[i][j].highest_val = -1000;
			ts->tp[i][j].touch_delay = -1000;
			ts->tp[i][j].hover_x = -1000;
			ts->tp[i][j].hover_y = -1000;
			ts->tp[i][j].hover_delay = HOVER_DEBOUNCE_DELAY;
			ts->tp[i][j].input_slot = -1;
		}
	}

	for (i = 0; i < MAX_TOUCH; i++)
		ts->slot_in_use[i] = 0;
}

static void cy8ctma395_ts_liftoff(struct cy8ctma395_ts_data *ts)
{
	int i;

	for (i = 0; i < MAX_TOUCH; i++) {
		if (ts->slot_in_use[i]) {
			input_mt_slot(ts->input, i);
			input_mt_report_slot_inactive(ts->input);
			ts->slot_in_use[i] = 0;
		}
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

/* Recursive flood fill for touch area fringe detection */
static void cy8ctma395_ts_area_fringe(struct cy8ctma395_ts_data *ts,
				      long *isum, long *jsum, int *tweight,
				      int i, int j, int cur_touch_id,
				      int depth)
{
	int powered;

	if (depth-- == 0)
		return;

	ts->invalid_matrix[i][j] = cur_touch_id;

	powered = pow_1_5(ts->matrix[i][j]);
	*tweight += powered;
	*isum += (long)powered * i;
	*jsum += (long)powered * j;

	/* Check adjacent cells if they're decreasing (part of same touch) */
	if (i > 0 && ts->invalid_matrix[i-1][j] != cur_touch_id &&
	    ts->matrix[i-1][j] >= LARGE_AREA_FRINGE &&
	    ts->matrix[i-1][j] < ts->matrix[i][j])
		cy8ctma395_ts_area_fringe(ts, isum, jsum, tweight,
					  i - 1, j, cur_touch_id, depth);

	if (i < X_AXIS_POINTS - 1 && ts->invalid_matrix[i+1][j] != cur_touch_id &&
	    ts->matrix[i+1][j] >= LARGE_AREA_FRINGE &&
	    ts->matrix[i+1][j] < ts->matrix[i][j])
		cy8ctma395_ts_area_fringe(ts, isum, jsum, tweight,
					  i + 1, j, cur_touch_id, depth);

	if (j > 0 && ts->invalid_matrix[i][j-1] != cur_touch_id &&
	    ts->matrix[i][j-1] >= LARGE_AREA_FRINGE &&
	    ts->matrix[i][j-1] < ts->matrix[i][j])
		cy8ctma395_ts_area_fringe(ts, isum, jsum, tweight,
					  i, j - 1, cur_touch_id, depth);

	if (j < Y_AXIS_POINTS - 1 && ts->invalid_matrix[i][j+1] != cur_touch_id &&
	    ts->matrix[i][j+1] >= LARGE_AREA_FRINGE &&
	    ts->matrix[i][j+1] < ts->matrix[i][j])
		cy8ctma395_ts_area_fringe(ts, isum, jsum, tweight,
					  i, j + 1, cur_touch_id, depth);
}

/* Recursive flood fill for main touch area detection */
static void cy8ctma395_ts_determine_area(struct cy8ctma395_ts_data *ts,
					 long *isum, long *jsum, int *tweight,
					 int i, int j,
					 int *mini, int *maxi,
					 int *minj, int *maxj,
					 int cur_touch_id, int *highest_val,
					 int depth)
{
	int powered;

	if (depth-- == 0)
		return;

	ts->invalid_matrix[i][j] = cur_touch_id;

	/* Track size of touch area */
	if (i < *mini) *mini = i;
	if (i > *maxi) *maxi = i;
	if (j < *minj) *minj = j;
	if (j > *maxj) *maxj = j;

	if (ts->matrix[i][j] > *highest_val)
		*highest_val = ts->matrix[i][j];

	powered = pow_1_5(ts->matrix[i][j]);
	*tweight += powered;
	*isum += (long)powered * i;
	*jsum += (long)powered * j;

	/* Check adjacent cells */
	if (i > 0 && ts->invalid_matrix[i-1][j] != cur_touch_id) {
		if (ts->matrix[i-1][j] >= LARGE_AREA_UNPRESS)
			cy8ctma395_ts_determine_area(ts, isum, jsum, tweight,
						     i - 1, j, mini, maxi,
						     minj, maxj,
						     cur_touch_id, highest_val,
						     depth);
		else if (ts->matrix[i-1][j] >= LARGE_AREA_FRINGE &&
			 ts->matrix[i-1][j] < ts->matrix[i][j])
			cy8ctma395_ts_area_fringe(ts, isum, jsum, tweight,
						  i - 1, j, cur_touch_id, depth);
	}

	if (i < X_AXIS_POINTS - 1 && ts->invalid_matrix[i+1][j] != cur_touch_id) {
		if (ts->matrix[i+1][j] >= LARGE_AREA_UNPRESS)
			cy8ctma395_ts_determine_area(ts, isum, jsum, tweight,
						     i + 1, j, mini, maxi,
						     minj, maxj,
						     cur_touch_id, highest_val,
						     depth);
		else if (ts->matrix[i+1][j] >= LARGE_AREA_FRINGE &&
			 ts->matrix[i+1][j] < ts->matrix[i][j])
			cy8ctma395_ts_area_fringe(ts, isum, jsum, tweight,
						  i + 1, j, cur_touch_id, depth);
	}

	if (j > 0 && ts->invalid_matrix[i][j-1] != cur_touch_id) {
		if (ts->matrix[i][j-1] >= LARGE_AREA_UNPRESS)
			cy8ctma395_ts_determine_area(ts, isum, jsum, tweight,
						     i, j - 1, mini, maxi,
						     minj, maxj,
						     cur_touch_id, highest_val,
						     depth);
		else if (ts->matrix[i][j-1] >= LARGE_AREA_FRINGE &&
			 ts->matrix[i][j-1] < ts->matrix[i][j])
			cy8ctma395_ts_area_fringe(ts, isum, jsum, tweight,
						  i, j - 1, cur_touch_id, depth);
	}

	if (j < Y_AXIS_POINTS - 1 && ts->invalid_matrix[i][j+1] != cur_touch_id) {
		if (ts->matrix[i][j+1] >= LARGE_AREA_UNPRESS)
			cy8ctma395_ts_determine_area(ts, isum, jsum, tweight,
						     i, j + 1, mini, maxi,
						     minj, maxj,
						     cur_touch_id, highest_val,
						     depth);
		else if (ts->matrix[i][j+1] >= LARGE_AREA_FRINGE &&
			 ts->matrix[i][j+1] < ts->matrix[i][j])
			cy8ctma395_ts_area_fringe(ts, isum, jsum, tweight,
						  i, j + 1, cur_touch_id, depth);
	}
}

static void cy8ctma395_ts_avg_filter(struct cy8ctma395_ts_data *ts,
				     struct cy8ctma395_touchpoint *t)
{
	int xsum, ysum;
	int total_div = 6;
	int prev_loc = t->prev_loc;
	int prevtpoint = ts->prevtpoint;
	int prev2tpoint = ts->prev2tpoint;

	xsum = 4 * t->unfiltered_x + 2 * ts->tp[prevtpoint][prev_loc].unfiltered_x;
	ysum = 4 * t->unfiltered_y + 2 * ts->tp[prevtpoint][prev_loc].unfiltered_y;

	if (ts->tp[prevtpoint][prev_loc].prev_loc >= 0) {
		int prev2_loc = ts->tp[prevtpoint][prev_loc].prev_loc;

		xsum += ts->tp[prev2tpoint][prev2_loc].unfiltered_x;
		ysum += ts->tp[prev2tpoint][prev2_loc].unfiltered_y;
		total_div = 7;
	}

	t->x = xsum / total_div;
	t->y = ysum / total_div;
}

static void cy8ctma395_ts_hover_debounce(struct cy8ctma395_ts_data *ts, int idx)
{
	struct cy8ctma395_touchpoint *t = &ts->tp[ts->tpoint][idx];
	int prev_loc = t->prev_loc;
	int prevtpoint = ts->prevtpoint;

	t->hover_delay = ts->tp[prevtpoint][prev_loc].hover_delay;

	if (abs(t->x - ts->tp[prevtpoint][prev_loc].hover_x) < HOVER_DEBOUNCE_RADIUS &&
	    abs(t->y - ts->tp[prevtpoint][prev_loc].hover_y) < HOVER_DEBOUNCE_RADIUS) {
		if (!t->hover_delay) {
			t->x = ts->tp[prevtpoint][prev_loc].hover_x;
			t->y = ts->tp[prevtpoint][prev_loc].hover_y;
		} else {
			t->hover_delay--;
		}

		if (ts->tp[prevtpoint][prev_loc].hover_delay != 1) {
			t->hover_x = ts->tp[prevtpoint][prev_loc].hover_x;
			t->hover_y = ts->tp[prevtpoint][prev_loc].hover_y;
		}
	} else {
		t->hover_delay = HOVER_DEBOUNCE_DELAY;
	}
}

static void cy8ctma395_ts_process_new_touch(struct cy8ctma395_ts_data *ts,
					    struct cy8ctma395_touchpoint *t)
{
	if (t->highest_val > ts->touch_delay_thresh) {
		t->tracking_id = ts->next_tracking_id++;
		if (ts->next_tracking_id > TRACKING_ID_MAX)
			ts->next_tracking_id = 0;

		if (t->highest_val <= ts->touch_initial_thresh)
			t->touch_delay = ts->touch_delay_count;
	} else {
		t->highest_val = 0;
	}
}

static int cy8ctma395_ts_calc_point(struct cy8ctma395_ts_data *ts)
{
	int i, j;
	int tpc = 0;
	int smallest_distance[MAX_TOUCH];
	int smallest_distance_loc[MAX_TOUCH];
	int tpoint;
	int max_val = 0;
	static unsigned long last_calc_print;

	/* Find max value in matrix for debug */
	for (i = 0; i < X_AXIS_POINTS; i++) {
		for (j = 0; j < Y_AXIS_POINTS; j++) {
			if (ts->matrix[i][j] > max_val)
				max_val = ts->matrix[i][j];
		}
	}

	if (printk_timed_ratelimit(&last_calc_print, 5000))
		pr_info("cy8ctma395: calc_point called, rows=%u, max_val=%d, thresh=%d\n",
			ts->rows_received, max_val, ts->touch_continue_thresh);

	if (ts->tp[ts->tpoint][0].x < -20) {
		/* Total liftoff occurred */
		ts->previoustpc = 0;
	} else {
		/* Rotate frame indices */
		ts->prev2tpoint = ts->prevtpoint;
		ts->prevtpoint = ts->tpoint;
		ts->tpoint++;
		if (ts->tpoint > 2)
			ts->tpoint = 0;
	}
	tpoint = ts->tpoint;

	/* Scan matrix for touches */
	memset(ts->invalid_matrix, 0, sizeof(ts->invalid_matrix));

	for (i = 0; i < X_AXIS_POINTS; i++) {
		for (j = 0; j < Y_AXIS_POINTS; j++) {
			if (tpc < MAX_TOUCH &&
			    ts->matrix[i][j] > ts->touch_continue_thresh &&
			    !ts->invalid_matrix[i][j]) {

				long isum = 0, jsum = 0;
				int tweight = 0;
				int mini = i, maxi = i, minj = j, maxj = j;
				int highest_val = ts->matrix[i][j];
				struct cy8ctma395_touchpoint *t;

				cy8ctma395_ts_determine_area(ts, &isum, &jsum,
							     &tweight, i, j,
							     &mini, &maxi,
							     &minj, &maxj,
							     tpc + 1,
							     &highest_val,
							     CY8CTMA395_FLOOD_DEPTH_MAX);

				if (tweight > 0) {
					t = &ts->tp[tpoint][tpc];
					t->pw = tweight;
					t->i = isum / tweight;
					t->j = jsum / tweight;
					t->touch_major = max(maxi - mini,
							     maxj - minj) *
							 PIXELS_PER_POINT;
					t->tracking_id = -1;
					t->prev_loc = -1;

					/* Convert to screen coordinates */
					t->x = (X_RESOLUTION - 1) -
					       (t->j * X_RESOLUTION /
						(Y_AXIS_POINTS - 1));
					t->y = (Y_RESOLUTION - 1) -
					       (t->i * Y_RESOLUTION /
						(X_AXIS_POINTS - 1));

					t->x = clamp(t->x, 0, X_RESOLUTION - 1);
					t->y = clamp(t->y, 0, Y_RESOLUTION - 1);

					t->unfiltered_x = t->x;
					t->unfiltered_y = t->y;
					t->highest_val = highest_val;
					t->touch_delay = 0;
					t->hover_x = t->x;
					t->hover_y = t->y;
					t->hover_delay = HOVER_DEBOUNCE_DELAY;
					t->input_slot = -1;

					pr_debug("cy8ctma395: touch detected at (%d,%d) val=%d weight=%d\n",
						t->x, t->y, highest_val, tweight);
					tpc++;
				}
			}
		}
	}

	/* Mark previously used slots for potential liftoff */
	for (i = 0; i < MAX_TOUCH; i++)
		if (ts->slot_in_use[i])
			ts->slot_in_use[i] = -1;

	/* Match touches to previous frame */
	for (i = 0; i < tpc; i++) {
		smallest_distance[i] = 1000000;
		smallest_distance_loc[i] = -1;

		for (j = 0; j < ts->previoustpc; j++) {
			if (ts->tp[ts->prevtpoint][j].highest_val) {
				int dx = ts->tp[tpoint][i].unfiltered_x -
					 ts->tp[ts->prevtpoint][j].unfiltered_x;
				int dy = ts->tp[tpoint][i].unfiltered_y -
					 ts->tp[ts->prevtpoint][j].unfiltered_y;
				int dist = dx * dx + dy * dy;

				if (dist < smallest_distance[i]) {
					smallest_distance[i] = dist;
					smallest_distance_loc[i] = j;
				}
			}
		}
	}

	/* Remove duplicate mappings */
	for (i = 0; i < tpc; i++) {
		for (j = i + 1; j < tpc; j++) {
			if (smallest_distance_loc[i] >= 0 &&
			    smallest_distance_loc[i] == smallest_distance_loc[j]) {
				if (smallest_distance[i] < smallest_distance[j])
					smallest_distance_loc[j] = -1;
				else
					smallest_distance_loc[i] = -1;
			}
		}
	}

	/* Assign tracking IDs */
	for (i = 0; i < tpc; i++) {
		struct cy8ctma395_touchpoint *t = &ts->tp[tpoint][i];

		if (smallest_distance_loc[i] >= 0) {
			int prev_idx = smallest_distance_loc[i];

			if (smallest_distance[i] > MAX_DELTA_SQ) {
				/* Large jump - treat as new touch */
				cy8ctma395_ts_process_new_touch(ts, t);
			} else {
				/* Continue existing touch */
				t->tracking_id =
					ts->tp[ts->prevtpoint][prev_idx].tracking_id;
				t->prev_loc = prev_idx;
				t->touch_delay =
					ts->tp[ts->prevtpoint][prev_idx].touch_delay;
				t->distance = smallest_distance[i];
				/*
				 * Reuse the MT slot from the previous frame so
				 * user space sees a single continuous contact
				 * (essential for drag/swipe gestures).
				 */
				t->input_slot =
					ts->tp[ts->prevtpoint][prev_idx].input_slot;

				cy8ctma395_ts_avg_filter(ts, t);
				cy8ctma395_ts_hover_debounce(ts, i);

				if (t->input_slot >= 0 &&
				    t->input_slot < MAX_TOUCH)
					ts->slot_in_use[t->input_slot] = 1;
			}
		} else {
			cy8ctma395_ts_process_new_touch(ts, t);
		}
	}

	/* Report touches */
	for (i = 0; i < tpc; i++) {
		struct cy8ctma395_touchpoint *t = &ts->tp[tpoint][i];

		if (t->highest_val && !t->touch_delay && t->tracking_id >= 0) {
			/*
			 * Continued touches kept their previous slot above.
			 * Only new touches need a fresh slot here.  Skip slots
			 * that are about to be lifted off (slot_in_use == -1)
			 * so the inactivate + activate don't collide on the
			 * same slot within one frame.
			 */
			if (t->input_slot < 0) {
				for (j = 0; j < MAX_TOUCH; j++) {
					if (ts->slot_in_use[j] == 0) {
						t->input_slot = j;
						ts->slot_in_use[j] = 1;
						break;
					}
				}
			}

			if (t->input_slot >= 0) {
				input_mt_slot(ts->input, t->input_slot);
				input_mt_report_slot_state(ts->input,
							   MT_TOOL_FINGER,
							   true);
				/* Note: tracking ID is handled by input_mt_report_slot_state() */
				input_report_abs(ts->input,
						 ABS_MT_POSITION_X, t->x);
				input_report_abs(ts->input,
						 ABS_MT_POSITION_Y, t->y);
				/*
				 * touch_major is the bounding-box width of the
				 * contact in pixels (PIXELS_PER_POINT per grid
				 * cell). A palm covering the full width can
				 * exceed Y_AXIS_POINTS * PIXELS_PER_POINT;
				 * clamp to the declared axis maximum so user-
				 * space readers do not see out-of-range values.
				 *
				 * pw is the area-integrated capacitance sum
				 * (sum of pow_1_5(matrix[i][j])) over the
				 * contact's bounding cells; for a wide contact
				 * it can run into the tens of thousands. Clamp
				 * to the declared ABS_MT_PRESSURE range.
				 */
				input_report_abs(ts->input,
						 ABS_MT_TOUCH_MAJOR,
						 clamp(t->touch_major, 0, 500));
				input_report_abs(ts->input,
						 ABS_MT_PRESSURE,
						 clamp(t->pw, 250, 2000));
			}
		} else if (t->touch_delay) {
			t->touch_delay--;
		}
	}

	/* Lift off unused slots */
	for (i = 0; i < MAX_TOUCH; i++) {
		if (ts->slot_in_use[i] == -1) {
			input_mt_slot(ts->input, i);
			input_mt_report_slot_inactive(ts->input);
			ts->slot_in_use[i] = 0;
		}
	}

	/*
	 * Sync the MT state - this generates BTN_TOUCH and single-touch
	 * ABS_X/ABS_Y events from the MT slot data for input handlers
	 * that need them.
	 */
	input_mt_sync_frame(ts->input);

	/*
	 * Always sync after a completed scan, even when tpc == 0, so that
	 * slot deactivations emitted by the liftoff loop above (and by
	 * input_mt_sync_frame's INPUT_MT_DROP_UNUSED cleanup) actually
	 * reach user space.
	 */
	input_sync(ts->input);
	if (tpc > 0)
		pr_debug("cy8ctma395: reporting %d touch(es)\n", tpc);

	ts->previoustpc = tpc;
	return tpc;
}

static bool cy8ctma395_ts_frame_valid(struct cy8ctma395_ts_data *ts,
				      unsigned int extras)
{
	if (ts->cline[0] == FRAME_START && ts->cline[1] == FRAME_ROW_DATA &&
	    ts->cidx == ROW_DATA_SIZE - extras)
		return true;

	if (ts->cline[0] == FRAME_START && ts->cline[1] == FRAME_SCAN_COMPLETE &&
	    ts->cidx > 4 && ts->cidx == (ts->cline[2] + 4 - extras))
		return true;

	return false;
}

static int cy8ctma395_ts_consume_frame(struct cy8ctma395_ts_data *ts)
{
	int i, ret = 0;

	if (ts->cline[1] == FRAME_SCAN_COMPLETE) {
		/* All row data received, calculate touches */
		ret = cy8ctma395_ts_calc_point(ts);
	}

	if (ts->cline[1] == FRAME_ROW_DATA) {
		int row = ts->cline[2] & 0x1F;

		/* Start of new scan - calculate touches from previous scan, then clear */
		if (ts->cline[2] & 0x80) {
			/* Calculate touches from the completed scan before clearing */
			if (ts->rows_received > 0) {
				ret = cy8ctma395_ts_calc_point(ts);
			}
			memset(ts->matrix, 0, sizeof(ts->matrix));
			ts->rows_received = 0;
		}

		/* Copy row data into matrix */
		if (row < X_AXIS_POINTS) {
			for (i = 0; i < Y_AXIS_POINTS; i++)
				ts->matrix[row][i] = ts->cline[i + 3];
			ts->rows_received++;
		}
	}

	ts->cidx = 0;
	return ret;
}

static void cy8ctma395_ts_put_byte(struct cy8ctma395_ts_data *ts, u8 byte)
{
	if (ts->cidx == 0 && byte != FRAME_START)
		return;

	/* Handle aborted transmissions */
	if (byte == FRAME_START && !cy8ctma395_ts_frame_valid(ts, 1))
		ts->cidx = 0;

	if (ts->cidx < sizeof(ts->cline))
		ts->cline[ts->cidx++] = byte;
}

static int cy8ctma395_ts_process_data(struct cy8ctma395_ts_data *ts,
				      const u8 *data, size_t len)
{
	size_t i;
	int touches = 0;
	static unsigned long last_frame_print;
	static int frame_count;

	for (i = 0; i < len; i++) {
		cy8ctma395_ts_put_byte(ts, data[i]);
		if (cy8ctma395_ts_frame_valid(ts, 0)) {
			frame_count++;
			if (printk_timed_ratelimit(&last_frame_print, 5000))
				pr_info("cy8ctma395: valid frame #%d, type=0x%02x, idx=%d\n",
					frame_count, ts->cline[1], ts->cidx);
			touches += cy8ctma395_ts_consume_frame(ts);
		}
	}

	return touches;
}

static size_t cy8ctma395_ts_receive_buf(struct serdev_device *serdev,
					const u8 *data, size_t count)
{
	struct cy8ctma395_ts_data *ts = serdev_device_get_drvdata(serdev);
	static unsigned long last_print;
	static size_t total_bytes;
	static bool first_data = true;

	total_bytes += count;
	if (printk_timed_ratelimit(&last_print, 5000))
		dev_info(&serdev->dev, "UART RX: %zu bytes total, last %zu bytes\n",
			 total_bytes, count);

	/* Print first 64 bytes received for debugging */
	if (first_data && count > 0) {
		first_data = false;
		print_hex_dump(KERN_INFO, "cy8ctma395 first RX: ", DUMP_PREFIX_OFFSET,
			       16, 1, data, min_t(size_t, count, 64), true);
	}

	/* Periodically analyze data format */
	{
		static unsigned long last_sample;
		static size_t ff_count;
		size_t i;

		/* Count 0xFF bytes (potential frame starts) */
		for (i = 0; i < count; i++) {
			if (data[i] == 0xFF)
				ff_count++;
		}

		if (printk_timed_ratelimit(&last_sample, 10000) && count >= 16) {
			pr_info("cy8ctma395: data analysis - 0xFF count: %zu, sample:\n", ff_count);
			print_hex_dump(KERN_INFO, "  ", DUMP_PREFIX_OFFSET,
				       16, 1, data, min_t(size_t, count, 64), true);
		}
	}

	/*
	 * Liftoff is handled inside cy8ctma395_ts_calc_point via the per-frame
	 * slot_in_use bookkeeping plus input_mt_sync_frame's drop-unused pass.
	 * Do NOT trigger a liftoff just because this serdev chunk did not
	 * happen to contain a scan-complete marker: at 4 Mbps a single scan
	 * spans many serdev callbacks, and spuriously lifting touches off
	 * every chunk breaks drag/swipe gestures (taps still register because
	 * the down+up arrive close in time, but a sustained contact is
	 * repeatedly torn down before user space can recognise it as motion).
	 *
	 * Hold state_lock across the parse so concurrent suspend / resume /
	 * remove paths cannot walk tp[][] mid-update.
	 */
	mutex_lock(&ts->state_lock);
	cy8ctma395_ts_process_data(ts, data, count);
	mutex_unlock(&ts->state_lock);

	return count;
}

static const struct serdev_device_ops cy8ctma395_ts_serdev_ops = {
	.receive_buf = cy8ctma395_ts_receive_buf,
};

static int cy8ctma395_ts_i2c_write(struct cy8ctma395_ts_data *ts, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };
	struct i2c_msg msg = {
		.addr = CY8CTMA395_I2C_ADDR,
		.flags = 0,
		.len = 2,
		.buf = buf,
	};

	return i2c_transfer(ts->i2c->adapter, &msg, 1);
}

static int cy8ctma395_ts_i2c_write_multi(struct cy8ctma395_ts_data *ts,
					 const u8 *data, size_t len)
{
	struct i2c_msg msg = {
		.addr = CY8CTMA395_I2C_ADDR,
		.flags = 0,
		.len = len,
		.buf = (u8 *)data,
	};

	return i2c_transfer(ts->i2c->adapter, &msg, 1);
}

static int cy8ctma395_ts_power_on(struct cy8ctma395_ts_data *ts)
{
	static const u8 init_seq1[] = { 0x31, 0x01, 0x08, 0x0C, 0x0D, 0x0A };
	struct device *dev = &ts->serdev->dev;
	int ret, retry = 0;

retry:
	/*
	 * Power-on sequence based on ts_srv userspace daemon (tssrv3l.c):
	 * 1. Assert reset (xres=1)
	 * 2. Enable VDD power (vdd=1)
	 * 3. Wait 50ms for voltage stabilization
	 * 4. Assert wake (wake=1) - BEFORE reset deassertion!
	 * 5. Deassert reset (xres=0)
	 * 6. Wait 50ms for controller boot
	 * 7. Deassert wake (wake=0)
	 * 8. Wait 50ms
	 * 9. Send I2C init sequence
	 * 10. Assert wake to start streaming (wake=1)
	 */

	/* 1. Assert reset */
	dev_info(dev, "GPIO: assert reset (logical 1)\n");
	gpiod_set_value_cansleep(ts->gpio_reset, 1);

	/* 2. Power on */
	ret = regulator_enable(ts->vdd);
	if (ret) {
		dev_err(dev, "Failed to enable vdd: %d\n", ret);
		return ret;
	}
	dev_info(dev, "VDD enabled\n");

	/* 3. Wait 50ms for voltage stabilization */
	usleep_range(50000, 55000);

	/* 4. Assert wake BEFORE deasserting reset */
	dev_info(dev, "GPIO: assert wake (logical 1)\n");
	gpiod_set_value_cansleep(ts->gpio_wake, 1);

	/* 5. Deassert reset */
	dev_info(dev, "GPIO: deassert reset (logical 0)\n");
	gpiod_set_value_cansleep(ts->gpio_reset, 0);

	/* 6. Wait 50ms for controller boot */
	usleep_range(50000, 55000);

	/* 7. Deassert wake */
	dev_info(dev, "GPIO: deassert wake (logical 0)\n");
	gpiod_set_value_cansleep(ts->gpio_wake, 0);

	/* 8. Wait 50ms */
	usleep_range(50000, 55000);

	/* Send I2C initialization sequence */
	ret = cy8ctma395_ts_i2c_write(ts, 0x08, 0x00);
	dev_info(dev, "I2C init write 0x08=0x00: ret=%d\n", ret);
	if (ret != 1) {
		if (retry++ < 3) {
			regulator_disable(ts->vdd);
			usleep_range(10000, 15000);
			dev_warn(dev, "TS wakeup retry #%d\n", retry);
			goto retry;
		}
		/*
		 * Wakeup write has failed 3 times in a row. Be consistent
		 * with the init_seq1 failure handling below and abort
		 * power-on rather than silently continuing with an
		 * unresponsive controller.
		 */
		dev_err(dev, "TS wakeup write failed after 3 retries: %d\n",
			ret);
		regulator_disable(ts->vdd);
		return ret < 0 ? ret : -EIO;
	}

	ret = cy8ctma395_ts_i2c_write_multi(ts, init_seq1, sizeof(init_seq1));
	dev_info(dev, "I2C init seq1: ret=%d\n", ret);
	if (ret != 1) {
		/*
		 * The init_seq1 transaction is what wakes the controller up
		 * into streaming mode. Earlier revisions of this driver let
		 * probe continue here on failure, which left ts->powered set
		 * but the device unconfigured and silent on the UART, hiding
		 * the real cause and producing a "successfully bound but
		 * non-functional" touchscreen. Treat any failure as fatal
		 * and return -EIO so the regulator/GPIO teardown in
		 * probe's error path runs.
		 */
		dev_err(dev, "Init seq1 failed: %d -- aborting power-on\n",
			ret);
		regulator_disable(ts->vdd);
		return ret < 0 ? ret : -EIO;
	}

	ret = cy8ctma395_ts_i2c_write(ts, 0x30, 0x0F);
	dev_info(dev, "I2C write 0x30=0x0F: ret=%d\n", ret);
	ret = cy8ctma395_ts_i2c_write(ts, 0x40, 0x02);
	dev_info(dev, "I2C write 0x40=0x02: ret=%d\n", ret);
	ret = cy8ctma395_ts_i2c_write(ts, 0x41, 0x10);
	dev_info(dev, "I2C write 0x41=0x10: ret=%d\n", ret);
	ret = cy8ctma395_ts_i2c_write(ts, 0x0A, 0x04);
	dev_info(dev, "I2C write 0x0A=0x04: ret=%d\n", ret);
	ret = cy8ctma395_ts_i2c_write(ts, 0x08, 0x03);
	dev_info(dev, "I2C write 0x08=0x03: ret=%d\n", ret);

	/* Assert wake to start streaming */
	dev_info(dev, "GPIO: assert wake to start streaming (logical 1)\n");
	gpiod_set_value_cansleep(ts->gpio_wake, 1);
	dev_info(dev, "Touchscreen power-on complete, waiting for UART data...\n");

	ts->powered = true;
	return 0;
}

static void cy8ctma395_ts_power_off(struct cy8ctma395_ts_data *ts)
{
	if (!ts->powered)
		return;

	regulator_disable(ts->vdd);

	/* Reset to stop data stream */
	gpiod_set_value_cansleep(ts->gpio_reset, 1);
	usleep_range(10000, 15000);
	gpiod_set_value_cansleep(ts->gpio_reset, 0);
	usleep_range(80000, 85000);

	ts->powered = false;
}

static int cy8ctma395_ts_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct cy8ctma395_ts_data *ts;
	struct i2c_adapter *i2c_adap;
	struct device_node *i2c_node;
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->serdev = serdev;
	serdev_device_set_drvdata(serdev, ts);

	/* Get I2C adapter from device tree */
	i2c_node = of_parse_phandle(dev->of_node, "i2c-bus", 0);
	if (!i2c_node) {
		dev_err(dev, "No i2c-bus specified\n");
		return -ENODEV;
	}

	i2c_adap = of_find_i2c_adapter_by_node(i2c_node);
	of_node_put(i2c_node);
	if (!i2c_adap)
		return -EPROBE_DEFER;

	ts->i2c = i2c_new_dummy_device(i2c_adap, CY8CTMA395_I2C_ADDR);
	i2c_put_adapter(i2c_adap);
	if (IS_ERR(ts->i2c)) {
		dev_err(dev, "Failed to create I2C device\n");
		return PTR_ERR(ts->i2c);
	}

	/* Get GPIOs */
	ts->gpio_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->gpio_reset)) {
		ret = PTR_ERR(ts->gpio_reset);
		dev_err(dev, "Failed to get reset GPIO: %d\n", ret);
		goto err_i2c;
	}

	ts->gpio_wake = devm_gpiod_get(dev, "wake", GPIOD_OUT_LOW);
	if (IS_ERR(ts->gpio_wake)) {
		ret = PTR_ERR(ts->gpio_wake);
		dev_err(dev, "Failed to get wake GPIO: %d\n", ret);
		goto err_i2c;
	}

	/* Get regulator */
	ts->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ts->vdd)) {
		ret = PTR_ERR(ts->vdd);
		dev_err(dev, "Failed to get vdd regulator: %d\n", ret);
		goto err_i2c;
	}

	/* Initialize thresholds */
	ts->touch_initial_thresh = TOUCH_INITIAL_THRESH;
	ts->touch_continue_thresh = TOUCH_CONTINUE_THRESH;
	ts->touch_delay_thresh = TOUCH_DELAY_THRESH;
	ts->touch_delay_count = TOUCH_DELAY_COUNT;

	/* Setup input device */
	ts->input = devm_input_allocate_device(dev);
	if (!ts->input) {
		ret = -ENOMEM;
		goto err_i2c;
	}

	ts->input->name = "HP TouchPad Touchscreen";
	ts->input->phys = "cy8ctma395/input0";
	ts->input->id.bustype = BUS_RS232;
	ts->input->id.vendor = 0x04b4;	/* Cypress */
	ts->input->id.product = 0x0395;
	ts->input->id.version = 0x0100;

	/* Single-touch axes - needed for pointer emulation */
	input_set_abs_params(ts->input, ABS_X, 0, X_RESOLUTION - 1, 2, 0);
	input_set_abs_params(ts->input, ABS_Y, 0, Y_RESOLUTION - 1, 1, 0);
	input_set_abs_params(ts->input, ABS_PRESSURE, 250, 2000, 0, 0);

	/* Multi-touch axes */
	input_set_abs_params(ts->input, ABS_MT_POSITION_X,
			     0, X_RESOLUTION - 1, 2, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y,
			     0, Y_RESOLUTION - 1, 1, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR,
			     PIXELS_PER_POINT, 500, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 250, 2000, 0, 0);

	ret = input_mt_init_slots(ts->input, MAX_TOUCH,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret) {
		dev_err(dev, "Failed to init MT slots: %d\n", ret);
		goto err_i2c;
	}

	/* Setup serdev */
	ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(dev, "Failed to open serdev: %d\n", ret);
		goto err_i2c;
	}

	/*
	 * Set client ops BEFORE baudrate to avoid race condition.
	 * Setting baudrate triggers msm_set_termios which starts RX DMA.
	 * If data arrives before client_ops is set, receive callback is NULL.
	 */
	serdev_device_set_client_ops(serdev, &cy8ctma395_ts_serdev_ops);
	serdev_device_set_flow_control(serdev, false);
	{
		unsigned int actual_baud;
		unsigned int delta;

		actual_baud = serdev_device_set_baudrate(serdev, 4000000);
		/*
		 * The CY8CTMA395 streams capacitance frames at 4 Mbps over
		 * UART. If the host controller cannot match that rate (or
		 * the closest hardware-selectable divisor is too far off),
		 * the bytestream parser below desynchronises and we report
		 * spurious touches. Reject anything outside +/- 1 % so a
		 * silent host-side fallback (e.g. msm_serial picking 921600)
		 * fails probe rather than producing junk.
		 */
		delta = actual_baud > 4000000 ?
			actual_baud - 4000000 : 4000000 - actual_baud;
		if (delta > 40000) {
			dev_err(dev,
				"UART baud out of tolerance: requested 4000000, got %u\n",
				actual_baud);
			ret = -ENODEV;
			goto err_serdev;
		}
		dev_info(dev, "Requested baud 4000000, actual baud %u\n",
			 actual_baud);
	}

	/* Initialize state */
	mutex_init(&ts->state_lock);
	cy8ctma395_ts_clear_arrays(ts);

	/* Power on touchscreen */
	ret = cy8ctma395_ts_power_on(ts);
	if (ret)
		goto err_serdev;

	/* Register input device */
	input_set_drvdata(ts->input, ts);
	ret = input_register_device(ts->input);
	if (ret) {
		dev_err(dev, "Failed to register input device: %d\n", ret);
		goto err_power;
	}

	dev_info(dev, "CY8CTMA395 touchscreen initialized\n");
	return 0;

err_power:
	/*
	 * Mirror the remove path: close serdev FIRST so no further
	 * receive_buf callbacks can race the power-off / state-array
	 * teardown, then take state_lock and power down.
	 */
	serdev_device_close(serdev);
	mutex_lock(&ts->state_lock);
	cy8ctma395_ts_power_off(ts);
	mutex_unlock(&ts->state_lock);
	goto err_i2c;
err_serdev:
	serdev_device_close(serdev);
err_i2c:
	i2c_unregister_device(ts->i2c);
	return ret;
}

static void cy8ctma395_ts_remove(struct serdev_device *serdev)
{
	struct cy8ctma395_ts_data *ts = serdev_device_get_drvdata(serdev);

	/*
	 * Close serdev BEFORE power_off so no further receive_buf callbacks
	 * can fire and walk tp[][] / matrix[] after the regulator + GPIOs
	 * have been torn down.
	 */
	serdev_device_close(serdev);
	mutex_lock(&ts->state_lock);
	cy8ctma395_ts_power_off(ts);
	mutex_unlock(&ts->state_lock);
	i2c_unregister_device(ts->i2c);
}

static int cy8ctma395_ts_suspend(struct device *dev)
{
	struct cy8ctma395_ts_data *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->state_lock);
	cy8ctma395_ts_liftoff(ts);
	cy8ctma395_ts_power_off(ts);
	mutex_unlock(&ts->state_lock);

	return 0;
}

static int cy8ctma395_ts_resume(struct device *dev)
{
	struct cy8ctma395_ts_data *ts = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&ts->state_lock);
	cy8ctma395_ts_clear_arrays(ts);
	ret = cy8ctma395_ts_power_on(ts);
	mutex_unlock(&ts->state_lock);

	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(cy8ctma395_ts_pm_ops,
				cy8ctma395_ts_suspend, cy8ctma395_ts_resume);

static const struct of_device_id cy8ctma395_ts_of_match[] = {
	{ .compatible = "cypress,cy8ctma395-ts" },
	{ }
};
MODULE_DEVICE_TABLE(of, cy8ctma395_ts_of_match);

static struct serdev_device_driver cy8ctma395_ts_driver = {
	.driver = {
		.name = CY8CTMA395_TS_NAME,
		.of_match_table = cy8ctma395_ts_of_match,
		.pm = pm_sleep_ptr(&cy8ctma395_ts_pm_ops),
	},
	.probe = cy8ctma395_ts_probe,
	.remove = cy8ctma395_ts_remove,
};
module_serdev_device_driver(cy8ctma395_ts_driver);

MODULE_AUTHOR("CyanogenMod Touchpad Project");
MODULE_DESCRIPTION("Cypress CY8CTMA395 Touchscreen serdev Driver");
MODULE_LICENSE("GPL");

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
 * Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/serdev.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/math64.h>
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
 * Module-parameter tunables, exported under
 *   /sys/module/cy8ctma395_ts/parameters/
 *
 * Provided for on-device A/B testing of the post-rewrite touch
 * pipeline against the pre-rewrite working driver. Every knob defaults
 * to the value the rewrite shipped, so the module loads with bit-
 * identical behaviour; the user can then flip any individual knob via
 * sysfs to isolate which behaviour change is responsible for a touch
 * regression (e.g. the Qt-evdevtouch swipe stutter) without
 * rebuilding the kernel.
 *
 * The fuzz knobs are pushed into the input device's absinfo at the
 * top of cy8ctma395_ts_calc_point() so writes via sysfs take effect on
 * the next reported event. The clamp / hover / avg_filter knobs are
 * read directly in the relevant code paths each scan.
 */
static unsigned int param_pressure_clamp = 1;
static unsigned int param_touch_major_clamp = 1;
static unsigned int param_avg_filter = 1;
static int param_fuzz_mt_x = 2;
static int param_fuzz_mt_y = 1;
static int param_fuzz_mt_pressure;
static int param_fuzz_st_x = 2;
static int param_fuzz_st_y = 1;
static int param_fuzz_st_pressure;
static unsigned int param_hover_radius = HOVER_DEBOUNCE_RADIUS;
static unsigned int param_hover_delay = HOVER_DEBOUNCE_DELAY;
static unsigned int param_single_emit_per_scan;
static unsigned int param_subcell_precision;

module_param_named(pressure_clamp, param_pressure_clamp, uint, 0644);
MODULE_PARM_DESC(pressure_clamp,
	"Clamp ABS_MT_PRESSURE to declared range [250,2000] (default 1). t->pw is the integrated pow_1_5 weight and runs into the tens of thousands, so with the clamp on the value is pinned at 2000 every reported frame; some userspace touch recognisers rely on a live pressure signal to keep a swipe alive (set 0 to forward raw t->pw).");

module_param_named(touch_major_clamp, param_touch_major_clamp, uint, 0644);
MODULE_PARM_DESC(touch_major_clamp,
	"Clamp ABS_MT_TOUCH_MAJOR to declared range [0,500] (default 1, set 0 to forward raw bounding-box width).");

module_param_named(avg_filter, param_avg_filter, uint, 0644);
MODULE_PARM_DESC(avg_filter,
	"3-frame weighted-average smoothing on touch coordinates (default 1). Adds ~3 frames of latency; set 0 to report unfiltered centroids per scan.");

module_param_named(fuzz_mt_x, param_fuzz_mt_x, int, 0644);
MODULE_PARM_DESC(fuzz_mt_x, "Fuzz on ABS_MT_POSITION_X (default 2)");
module_param_named(fuzz_mt_y, param_fuzz_mt_y, int, 0644);
MODULE_PARM_DESC(fuzz_mt_y, "Fuzz on ABS_MT_POSITION_Y (default 1)");
module_param_named(fuzz_mt_pressure, param_fuzz_mt_pressure, int, 0644);
MODULE_PARM_DESC(fuzz_mt_pressure, "Fuzz on ABS_MT_PRESSURE (default 0)");

module_param_named(fuzz_st_x, param_fuzz_st_x, int, 0644);
MODULE_PARM_DESC(fuzz_st_x,
	"Fuzz on ABS_X (the single-touch axis Qt evdevtouch / xf86-input-evdev read; default 2 matches the 6de96bdc restore; 0 forwards every centroid wobble unfiltered).");
module_param_named(fuzz_st_y, param_fuzz_st_y, int, 0644);
MODULE_PARM_DESC(fuzz_st_y, "Fuzz on ABS_Y (default 1)");
module_param_named(fuzz_st_pressure, param_fuzz_st_pressure, int, 0644);
MODULE_PARM_DESC(fuzz_st_pressure, "Fuzz on ABS_PRESSURE (default 0)");

module_param_named(hover_radius, param_hover_radius, uint, 0644);
MODULE_PARM_DESC(hover_radius,
	"Pixel dead-zone for the hover-debounce freeze (default 2). Set 0 to disable the freeze entirely; a stationary finger's reported coords then track the live centroid jitter.");
module_param_named(hover_delay, param_hover_delay, uint, 0644);
MODULE_PARM_DESC(hover_delay,
	"Stationary frames before a contact's reported coords lock to its hover_x/hover_y (default 30).");

module_param_named(single_emit_per_scan, param_single_emit_per_scan, uint, 0644);
MODULE_PARM_DESC(single_emit_per_scan,
	"Reset rows_received / matrix inside the FRAME_SCAN_COMPLETE handler so the start-of-next-row-0 path does not re-fire cy8ctma395_ts_calc_point() on the same scan (default 0). On-device evtest shows the current double-fire produces a 1-ms-then-9-ms paired event pattern that breaks slide-to-unlock recognisers; set 1 to emit exactly one report per scan.");

module_param_named(subcell_precision, param_subcell_precision, uint, 0644);
MODULE_PARM_DESC(subcell_precision,
	"Compute t->x / t->y directly from the weighted matrix sums (jsum * screen_w / (tweight * (Y_AXIS_POINTS - 1))) instead of via the integer-truncated centroid cell index t->j / t->i (default 0). The current truncation rounds the centroid to one of 30 x 40 integer cells, so a slow finger that moves less than ~26 px stays at the same reported x/y across many scans and then snaps a full cell -- visible to userspace as a stationary touch interrupted by a periodic jump. The ts_srv userspace reference performed this division in float for the same reason. Set 1 to evaluate the higher-precision integer form on hardware.");


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

	/*
	 * Pre-allocated work stack for the iterative flood-fill in
	 * cy8ctma395_ts_flood_fill(). Bounded by MATRIX_SIZE = 1200
	 * cells since we never revisit a cell (invalid_matrix marks it
	 * before push). 6 bytes per entry x 1200 = 7.2 KiB of struct
	 * footprint, kept here rather than on the stack because the
	 * worst-case adversarial UART payload could otherwise push the
	 * recursive variant past ARM's 8 KiB THREAD_SIZE.
	 *
	 * Access is serialised by state_lock so two RX bursts cannot
	 * stomp on the same stack.
	 */
	struct {
		u16 i;
		u16 j;
		bool fringe;
	} ff_stack[MATRIX_SIZE];

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
	 * Screen resolution. Filled at probe from
	 * touchscreen_parse_properties() (touchscreen-size-x/y in DT);
	 * defaults to X_RESOLUTION x Y_RESOLUTION (1024 x 768; the HP
	 * TouchPad panel) when the DT does not override them. Used by
	 * the centroid -> screen-coord conversion in calc_point so a
	 * board with a different panel can adjust without a driver
	 * patch.
	 */
	unsigned int screen_w;
	unsigned int screen_h;

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
			ts->tp[i][j].hover_delay = param_hover_delay;
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

/*
 * Iterative flood fill on the capacitance matrix. Replaces the prior
 * recursive cy8ctma395_ts_determine_area() + cy8ctma395_ts_area_fringe()
 * pair, which (even with a depth cap) put ~150-byte frames on the
 * kernel stack and could exceed ARM's 8 KiB THREAD_SIZE under
 * pathological UART data.
 *
 * The fill has two semantics, kept in a single function so neighbours
 * can be re-pushed easily across them:
 *
 *   fringe == false  --  main "press" area, equivalent to the old
 *                        determine_area(). Tracks the bounding box
 *                        (mini/maxi/minj/maxj) and the highest cell
 *                        value, and admits any neighbour >= UNPRESS
 *                        as another main-area cell, or any neighbour
 *                        >= FRINGE && < this cell as a fringe cell.
 *
 *   fringe == true   --  decreasing "fringe" cells around the area,
 *                        equivalent to the old area_fringe(). Only
 *                        admits neighbours >= FRINGE && < this cell.
 *
 * The work stack lives in ts->ff_stack (allocated once at probe; sized
 * for the worst case of every matrix cell being queued exactly once).
 * cur_touch_id is written into ts->invalid_matrix on PUSH (not on POP)
 * so the same cell can never be queued twice -- this bounds the stack
 * depth at MATRIX_SIZE.
 *
 * Caller holds state_lock (the RX path; suspend; remove).
 */
static void cy8ctma395_ts_flood_fill(struct cy8ctma395_ts_data *ts,
				     long *isum, long *jsum, int *tweight,
				     int start_i, int start_j,
				     int *mini, int *maxi,
				     int *minj, int *maxj,
				     int cur_touch_id, int *highest_val)
{
	unsigned int top = 0;

	ts->invalid_matrix[start_i][start_j] = cur_touch_id;
	ts->ff_stack[top].i = start_i;
	ts->ff_stack[top].j = start_j;
	ts->ff_stack[top].fringe = false;
	top++;

	while (top > 0) {
		int i, j, dir;
		bool fringe;
		int powered;
		u8 cell;

		top--;
		i      = ts->ff_stack[top].i;
		j      = ts->ff_stack[top].j;
		fringe = ts->ff_stack[top].fringe;
		cell   = ts->matrix[i][j];

		powered = pow_1_5(cell);
		*tweight += powered;
		*isum += (long)powered * i;
		*jsum += (long)powered * j;

		if (!fringe) {
			if (i < *mini) *mini = i;
			if (i > *maxi) *maxi = i;
			if (j < *minj) *minj = j;
			if (j > *maxj) *maxj = j;
			if (cell > *highest_val)
				*highest_val = cell;
		}

		/* Walk the 4-connected neighbourhood */
		for (dir = 0; dir < 4; dir++) {
			static const int di[4] = { -1,  1,  0,  0 };
			static const int dj[4] = {  0,  0, -1,  1 };
			int ni = i + di[dir];
			int nj = j + dj[dir];
			u8 ncell;
			bool nfringe;

			if (ni < 0 || ni >= X_AXIS_POINTS ||
			    nj < 0 || nj >= Y_AXIS_POINTS)
				continue;
			if (ts->invalid_matrix[ni][nj] == cur_touch_id)
				continue;

			ncell = ts->matrix[ni][nj];

			if (!fringe && ncell >= LARGE_AREA_UNPRESS) {
				nfringe = false;
			} else if (ncell >= LARGE_AREA_FRINGE && ncell < cell) {
				nfringe = true;
			} else {
				continue;
			}

			if (WARN_ON_ONCE(top >= MATRIX_SIZE))
				return;
			ts->invalid_matrix[ni][nj] = cur_touch_id;
			ts->ff_stack[top].i = ni;
			ts->ff_stack[top].j = nj;
			ts->ff_stack[top].fringe = nfringe;
			top++;
		}
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

	/*
	 * Runtime kill switch: leave t->x / t->y at the unfiltered centroid
	 * (already set by the matrix-scan loop in calc_point) so a userspace
	 * touch recogniser sees the raw per-scan trajectory rather than a
	 * 3-frame weighted blend that lags by ~3 frames.
	 */
	if (!param_avg_filter)
		return;

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

	if (abs(t->x - ts->tp[prevtpoint][prev_loc].hover_x) < (int)param_hover_radius &&
	    abs(t->y - ts->tp[prevtpoint][prev_loc].hover_y) < (int)param_hover_radius) {
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
		t->hover_delay = param_hover_delay;
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

	/*
	 * Push the current fuzz module parameters into the input device's
	 * absinfo. The input core re-reads absinfo->fuzz on every
	 * input_handle_event() call, so a sysfs write to
	 *   /sys/module/cy8ctma395_ts/parameters/fuzz_*
	 * takes effect on the next reported event. Cheap (~6 integer
	 * stores per scan) and avoids the bookkeeping of a separate
	 * parameter-set callback to find the input device.
	 */
	if (likely(ts->input->absinfo)) {
		ts->input->absinfo[ABS_X].fuzz = param_fuzz_st_x;
		ts->input->absinfo[ABS_Y].fuzz = param_fuzz_st_y;
		ts->input->absinfo[ABS_PRESSURE].fuzz = param_fuzz_st_pressure;
		ts->input->absinfo[ABS_MT_POSITION_X].fuzz = param_fuzz_mt_x;
		ts->input->absinfo[ABS_MT_POSITION_Y].fuzz = param_fuzz_mt_y;
		ts->input->absinfo[ABS_MT_PRESSURE].fuzz = param_fuzz_mt_pressure;
	}

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

				cy8ctma395_ts_flood_fill(ts, &isum, &jsum,
							 &tweight, i, j,
							 &mini, &maxi,
							 &minj, &maxj,
							 tpc + 1,
							 &highest_val);

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

					/*
					 * Convert to screen coordinates.
					 *
					 * Default form preserves the historical
					 * driver math: divide the weighted sum
					 * by tweight to get an integer matrix
					 * cell index (0..29 / 0..39) and then
					 * map that to screen pixels. This snaps
					 * x / y to a 26-pixel grid; a slow finger
					 * that moves less than one matrix cell
					 * holds the same reported value for many
					 * scans then jumps a full cell at once.
					 *
					 * subcell_precision=1 collapses the two
					 * divisions into one over the full
					 * weighted sum:
					 *   x = (screen_w - 1) -
					 *       jsum * screen_w
					 *       / (tweight * (Y_AXIS_POINTS - 1))
					 * preserving the sub-cell information
					 * that the centroid-then-quantise form
					 * threw away. Uses 64-bit intermediates
					 * because tweight runs into the tens of
					 * thousands and jsum * screen_w can
					 * exceed 2^31 on a wide contact. ts_srv
					 * userspace performed the equivalent in
					 * float.
					 */
					if (param_subcell_precision) {
						/*
						 * div_s64() because plain `/`
						 * on s64 lowers to
						 * __aeabi_ldivmod which the
						 * kernel does not link in on
						 * ARM32. The divisor
						 * (tweight * (axis_points-1))
						 * fits in s32: tweight peaks
						 * around the matrix-wide
						 * pow_1_5 sum (~ 4 M for an
						 * extreme palm) and the
						 * (axis - 1) factor is < 40.
						 */
						s64 x_num = (s64)jsum *
							    ts->screen_w;
						s32 x_den = tweight *
							    (Y_AXIS_POINTS - 1);
						s64 y_num = (s64)isum *
							    ts->screen_h;
						s32 y_den = tweight *
							    (X_AXIS_POINTS - 1);

						t->x = (ts->screen_w - 1) -
						       (int)div_s64(x_num,
								    x_den);
						t->y = (ts->screen_h - 1) -
						       (int)div_s64(y_num,
								    y_den);
					} else {
						t->x = (ts->screen_w - 1) -
						       (t->j * ts->screen_w /
							(Y_AXIS_POINTS - 1));
						t->y = (ts->screen_h - 1) -
						       (t->i * ts->screen_h /
							(X_AXIS_POINTS - 1));
					}

					t->x = clamp(t->x, 0,
						     (int)ts->screen_w - 1);
					t->y = clamp(t->y, 0,
						     (int)ts->screen_h - 1);

					t->unfiltered_x = t->x;
					t->unfiltered_y = t->y;
					t->highest_val = highest_val;
					t->touch_delay = 0;
					t->hover_x = t->x;
					t->hover_y = t->y;
					t->hover_delay = param_hover_delay;
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
						 param_touch_major_clamp ?
						 clamp(t->touch_major, 0, 500) :
						 t->touch_major);
				input_report_abs(ts->input,
						 ABS_MT_PRESSURE,
						 param_pressure_clamp ?
						 clamp(t->pw, 250, 2000) :
						 t->pw);
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

		/*
		 * Without resetting here, rows_received stays at X_AXIS_POINTS
		 * across the SCAN_COMPLETE -> next-scan-start-row gap, so the
		 * `if (ts->rows_received > 0)` guard in the FRAME_ROW_DATA path
		 * below re-fires cy8ctma395_ts_calc_point() on the still-
		 * populated matrix of the scan we just emitted.  On-device
		 * evtest shows that produces a paired event pattern (two
		 * SYN_REPORTs ~1 ms apart followed by a ~9 ms gap rather than
		 * a steady ~10 ms cadence): both calc_point() calls operate on
		 * the same matrix, but the frame-rotation state (tpoint /
		 * prevtpoint / prev2tpoint) advances between them, so the
		 * avg_filter blends with a different `prev` and emits a
		 * slightly different X / Y / pressure than the first call.
		 * Userspace slide-to-unlock recognisers parse the gap pattern
		 * as "move, stop, move, stop" and cancel the gesture.
		 *
		 * Gated behind a module parameter for on-device A/B
		 * comparison.  When the knob proves out, fold this into the
		 * unconditional consume_frame path and drop the kparam.
		 */
		if (param_single_emit_per_scan) {
			memset(ts->matrix, 0, sizeof(ts->matrix));
			ts->rows_received = 0;
		}
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

		/*
		 * Copy row data into matrix. The cline[2] row index is a
		 * 5-bit value (0..31) but the matrix only has X_AXIS_POINTS
		 * (30) rows. Indices 30/31 indicate either a desync against
		 * the controller's frame format or noise on the UART; drop
		 * the row rather than write past the matrix, and warn
		 * ratelimited so a misconfigured controller does not log-
		 * spam but is still visible at all.
		 */
		if (row < X_AXIS_POINTS) {
			for (i = 0; i < Y_AXIS_POINTS; i++)
				ts->matrix[row][i] = ts->cline[i + 3];
			ts->rows_received++;
		} else {
			dev_warn_ratelimited(&ts->serdev->dev,
				"row index %d out of range (>= %d), frame dropped\n",
				row, X_AXIS_POINTS);
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

	for (i = 0; i < len; i++) {
		cy8ctma395_ts_put_byte(ts, data[i]);
		if (cy8ctma395_ts_frame_valid(ts, 0))
			touches += cy8ctma395_ts_consume_frame(ts);
	}

	return touches;
}

static size_t cy8ctma395_ts_receive_buf(struct serdev_device *serdev,
					const u8 *data, size_t count)
{
	struct cy8ctma395_ts_data *ts = serdev_device_get_drvdata(serdev);

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

	/*
	 * Register-configuration writes following init_seq1. Same anti-
	 * pattern argument applies as above: silently continuing here
	 * left ts->powered = true but the controller half-configured;
	 * downstream symptom was a "successfully bound" touchscreen that
	 * streamed garbage frames. Treat any failure as fatal.
	 */
	struct {
		u8 reg;
		u8 val;
	} regs[] = {
		{ 0x30, 0x0f },
		{ 0x40, 0x02 },
		{ 0x41, 0x10 },
		{ 0x0a, 0x04 },
		{ 0x08, 0x03 },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = cy8ctma395_ts_i2c_write(ts, regs[i].reg, regs[i].val);
		if (ret != 1) {
			dev_err(dev,
				"I2C write %02x=%02x failed: %d -- aborting power-on\n",
				regs[i].reg, regs[i].val, ret);
			regulator_disable(ts->vdd);
			return ret < 0 ? ret : -EIO;
		}
	}

	/* Assert wake to start streaming */
	dev_dbg(dev, "GPIO: assert wake to start streaming (logical 1)\n");
	gpiod_set_value_cansleep(ts->gpio_wake, 1);
	dev_dbg(dev, "Touchscreen power-on complete, waiting for UART data...\n");

	ts->powered = true;
	return 0;
}

static void cy8ctma395_ts_power_off(struct cy8ctma395_ts_data *ts)
{
	if (!ts->powered)
		return;

	/*
	 * Mirror the power-on sequence in reverse to avoid back-powering
	 * the controller through its CMOS inputs:
	 *
	 *   1. deassert WAKE (the last GPIO we drove high in power_on),
	 *   2. assert RESET so the chip stops driving its own outputs,
	 *   3. only THEN drop VDD.
	 *
	 * The previous order (regulator_disable() first, GPIOs still high)
	 * dropped VDD while WAKE was being driven to logic high; with VDD
	 * collapsing under a CMOS input held above VDD + 0.3 V the chip
	 * is out of every Cypress PSoC absolute-maximum spec and at risk
	 * of latch-up. This runs on every suspend, remove and
	 * power-on-failure path.
	 */
	gpiod_set_value_cansleep(ts->gpio_wake, 0);
	gpiod_set_value_cansleep(ts->gpio_reset, 1);
	usleep_range(1000, 1500);

	regulator_disable(ts->vdd);

	/* Hold reset low after the rail collapses */
	usleep_range(10000, 15000);
	gpiod_set_value_cansleep(ts->gpio_reset, 0);
	usleep_range(80000, 85000);

	ts->powered = false;
}

static int cy8ctma395_ts_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct cy8ctma395_ts_data *ts;
	struct device_node *i2c_node;
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->serdev = serdev;
	serdev_device_set_drvdata(serdev, ts);

	/*
	 * Resolve the I2C sibling. The CY8CTMA395 has two host-side
	 * signalling paths: a 4 Mbps UART (this serdev_device, the
	 * primary data stream) and an I2C slave at 0x67 used for
	 * init/power. The two are described as separate DT nodes:
	 *
	 *   serial { touchscreen { compatible = "cypress,cy8ctma395";
	 *                          cypress,i2c = <&cy8c_i2c>; ... }; };
	 *   i2c    { touchscreen@67 { compatible = "cypress,cy8ctma395";
	 *                             reg = <0x67>; }; };
	 *
	 * The companion cy8ctma395_i2c_driver below binds to the i2c
	 * sibling and parks the resulting struct i2c_client in
	 * dev->driver_data. We dereference that here.
	 *
	 * Earlier revisions used a single non-standard "i2c-bus" phandle
	 * pointing at the whole I2C adapter and called
	 * i2c_new_dummy_device(adapter, 0x67) to manufacture a hidden
	 * client. That bypassed standard DT enumeration / i2cdetect /
	 * collision checking and was flagged in review; switch to the
	 * proper two-node pattern.
	 */
	i2c_node = of_parse_phandle(dev->of_node, "cypress,i2c", 0);
	if (!i2c_node)
		return dev_err_probe(dev, -EINVAL,
				     "missing cypress,i2c phandle to I2C sibling\n");

	ts->i2c = of_find_i2c_device_by_node(i2c_node);
	of_node_put(i2c_node);
	if (!ts->i2c)
		return -EPROBE_DEFER;

	/*
	 * The I2C sibling driver may have created the i2c_client but
	 * not yet have completed bind. Wait until it has.
	 */
	if (!ts->i2c->dev.driver) {
		put_device(&ts->i2c->dev);
		return -EPROBE_DEFER;
	}

	/*
	 * Force PM ordering: cy8ctma395_ts_resume() drives I2C writes
	 * via ts->i2c as part of power-on. dpm_list orders by
	 * registration, with no awareness of the cross-bus
	 * serdev<->i2c relationship we invent here. Without a
	 * device_link, on some boots the I2C client (and its adapter)
	 * is still suspended when our resume runs, the I2C writes
	 * return -ENXIO, the fatal-init handling promotes that to
	 * permanent disable.
	 *
	 * DL_FLAG_AUTOREMOVE_CONSUMER drops the link on our unbind.
	 * DL_FLAG_PM_RUNTIME ensures the supplier is runtime-resumed
	 * before this consumer runs.
	 */
	if (!device_link_add(dev, &ts->i2c->dev,
			     DL_FLAG_AUTOREMOVE_CONSUMER |
			     DL_FLAG_PM_RUNTIME)) {
		put_device(&ts->i2c->dev);
		return dev_err_probe(dev, -ENODEV,
				     "device_link to I2C sibling failed\n");
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
	/*
	 * BUS_HOST: the controller talks to us over an on-SoC UART via
	 * the serdev bus; there is no physical RS-232 connector. Matches
	 * other serdev-attached input drivers.
	 */
	ts->input->id.bustype = BUS_HOST;
	ts->input->id.vendor = 0x04b4;	/* Cypress */
	ts->input->id.product = 0x0395;
	ts->input->id.version = 0x0100;

	/*
	 * Multi-touch axes. input_mt_init_slots(INPUT_MT_DIRECT) below
	 * synthesises the corresponding ABS_X / ABS_Y / ABS_PRESSURE
	 * single-touch axes from these for pointer-emulation, so
	 * declaring them here would be dead code (the MT init would
	 * overwrite anything we set first).
	 */
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

	/*
	 * Let touchscreen_parse_properties() override the defaulted
	 * ABS_MT_POSITION_X / Y maxima with the standard touchscreen
	 * bindings (touchscreen-size-x / -y, touchscreen-inverted-x / -y,
	 * touchscreen-swapped-x-y) so a board with a different panel
	 * does not need a driver patch. Cache the result for the
	 * centroid -> screen-coord conversion in calc_point.
	 */
	touchscreen_parse_properties(ts->input, true, NULL);
	ts->screen_w = input_abs_get_max(ts->input, ABS_MT_POSITION_X) + 1;
	ts->screen_h = input_abs_get_max(ts->input, ABS_MT_POSITION_Y) + 1;

	/*
	 * Restore the single-touch fuzz values that copy_abs() in
	 * input_mt_init_slots() strips to 0. Userspace MT-aware stacks
	 * (Wayland/libinput) ignore the ST axes, but legacy single-touch
	 * paths (Qt's evdevtouch plugin, xf86-input-evdev) read ABS_X /
	 * ABS_Y / ABS_PRESSURE directly and rely on fuzz to suppress
	 * sub-cell centroid jitter -- without it, a stationary finger
	 * looks like a continuous swipe and tap-as-click recognisers
	 * cancel the click.
	 *
	 * Run AFTER touchscreen_parse_properties() so the fuzz survives
	 * any DT-driven axis swap / invert / resize. Use the same
	 * (small) fuzz values as the MT axes; the centroid is computed
	 * to sub-pixel precision so a fuzz of 1-2 pixels is well below
	 * "real" finger movement.
	 */
	input_abs_set_fuzz(ts->input, ABS_X, 2);
	input_abs_set_fuzz(ts->input, ABS_Y, 1);
	input_abs_set_fuzz(ts->input, ABS_PRESSURE, 0);

	/*
	 * Initialise state BEFORE the serdev RX callback can ever fire.
	 * serdev_device_set_client_ops() below publishes the receive_buf
	 * pointer to the serdev controller; once published, a queued RX
	 * burst can invoke cy8ctma395_ts_receive_buf() on a foreign worker
	 * at any moment, and its first action is mutex_lock(&ts->state_lock).
	 * Doing mutex_init() after that publication races the RX path on
	 * zero-fill memory (CONFIG_DEBUG_MUTEXES BUGs; otherwise the
	 * wait_lock can be torn). clear_arrays() must precede it for the
	 * same reason: the RX path consumes tp[][].input_slot expecting
	 * the -1 sentinel rather than the 0 left by devm_kzalloc.
	 */
	mutex_init(&ts->state_lock);
	cy8ctma395_ts_clear_arrays(ts);

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
		int actual_baud;
		unsigned int delta;

		/*
		 * serdev_device_set_baudrate() returns a signed int -- a
		 * negative errno on hard failure, otherwise the actual
		 * baud the controller settled on. Test < 0 BEFORE the
		 * unsigned tolerance arithmetic so a -ENODEV does not
		 * wrap into a huge "out-of-tolerance" delta and produce
		 * a misleading error.
		 */
		actual_baud = serdev_device_set_baudrate(serdev, 4000000);
		if (actual_baud < 0) {
			ret = actual_baud;
			dev_err(dev, "set_baudrate failed: %d\n", ret);
			goto err_serdev;
		}
		/*
		 * The CY8CTMA395 streams capacitance frames at 4 Mbps. If
		 * the host controller cannot match that rate (or the closest
		 * hardware-selectable divisor is too far off), the bytestream
		 * parser below desynchronises and we report spurious touches.
		 * Reject anything outside +/- 1 %.
		 */
		delta = actual_baud > 4000000 ?
			actual_baud - 4000000 : 4000000 - actual_baud;
		if (delta > 40000) {
			dev_err(dev,
				"UART baud out of tolerance: requested 4000000, got %d\n",
				actual_baud);
			ret = -ENODEV;
			goto err_serdev;
		}
		dev_dbg(dev, "Requested baud 4000000, actual baud %d\n",
			actual_baud);
	}

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
	/*
	 * Drop the of_find_i2c_device_by_node() reference. The i2c_client
	 * itself is owned by cy8ctma395_i2c_driver and lives across our
	 * unbind.
	 */
	put_device(&ts->i2c->dev);
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
	put_device(&ts->i2c->dev);
}

static int cy8ctma395_ts_suspend(struct device *dev)
{
	struct cy8ctma395_ts_data *ts = dev_get_drvdata(dev);
	struct serdev_device *serdev = ts->serdev;

	/*
	 * Stop the RX path BEFORE powering off and clearing state. With
	 * the serdev port still open, tty-layer buffered bytes (or noise
	 * from the chip's collapsing VDD on its TX line) keep arriving
	 * at our receive_buf, which then walks tp[][] / matrix[] after
	 * liftoff cleared them and emits phantom touches to userspace
	 * right around sleep. The matching reopen runs in resume below;
	 * the parser state (cline / cidx / rows_received) is reset there
	 * too so any half-frame from before suspend cannot stitch onto
	 * the first real bytes after resume.
	 */
	serdev_device_close(serdev);

	mutex_lock(&ts->state_lock);
	cy8ctma395_ts_liftoff(ts);
	cy8ctma395_ts_power_off(ts);
	ts->cidx = 0;
	ts->rows_received = 0;
	mutex_unlock(&ts->state_lock);

	return 0;
}

static int cy8ctma395_ts_resume(struct device *dev)
{
	struct cy8ctma395_ts_data *ts = dev_get_drvdata(dev);
	struct serdev_device *serdev = ts->serdev;
	int ret;

	mutex_lock(&ts->state_lock);
	cy8ctma395_ts_clear_arrays(ts);
	ret = cy8ctma395_ts_power_on(ts);
	mutex_unlock(&ts->state_lock);
	if (ret)
		return ret;

	ret = serdev_device_open(serdev);
	if (ret)
		return ret;

	serdev_device_set_client_ops(serdev, &cy8ctma395_ts_serdev_ops);
	serdev_device_set_flow_control(serdev, false);
	serdev_device_set_baudrate(serdev, 4000000);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(cy8ctma395_ts_pm_ops,
				cy8ctma395_ts_suspend, cy8ctma395_ts_resume);

static const struct of_device_id cy8ctma395_ts_of_match[] = {
	{ .compatible = "cypress,cy8ctma395" },
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

/*
 * Companion I2C driver. The CY8CTMA395 is described in DT as TWO nodes
 * sharing one compatible string -- a serdev child of the UART (the
 * primary data path) and an I2C client at address 0x67 under the I2C
 * controller (used by the serdev driver for init / power-on /
 * register configuration only). This stub binds to the I2C node so
 * the framework creates a real, DT-described struct i2c_client (vs
 * the prior i2c_new_dummy_device(adapter, 0x67) hack); the serdev
 * driver then looks the client up via of_find_i2c_device_by_node()
 * through the cypress,i2c phandle.
 *
 * No actual I2C traffic happens here -- all writes are issued from
 * the serdev side. The driver core's bind state is what we care
 * about: it serves as both "the i2c client exists" and "the i2c
 * controller is bound" gate for the serdev probe and for the PM
 * device_link.
 */
static int cy8ctma395_i2c_probe(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id cy8ctma395_i2c_of_match[] = {
	{ .compatible = "cypress,cy8ctma395" },
	{ }
};
MODULE_DEVICE_TABLE(of, cy8ctma395_i2c_of_match);

static struct i2c_driver cy8ctma395_i2c_driver = {
	.driver = {
		.name = "cy8ctma395-i2c",
		.of_match_table = cy8ctma395_i2c_of_match,
	},
	.probe = cy8ctma395_i2c_probe,
};

static int __init cy8ctma395_init(void)
{
	int ret;

	/*
	 * Order matters: register the i2c stub FIRST so it is available
	 * when the serdev probe runs of_find_i2c_device_by_node() against
	 * the cypress,i2c phandle. If the i2c side has not bound yet when
	 * the serdev probe fires, the serdev probe returns -EPROBE_DEFER
	 * and the driver core retries -- so registration order is not
	 * strictly required for correctness, but it minimises retries.
	 */
	ret = i2c_add_driver(&cy8ctma395_i2c_driver);
	if (ret)
		return ret;

	ret = serdev_device_driver_register(&cy8ctma395_ts_driver);
	if (ret)
		i2c_del_driver(&cy8ctma395_i2c_driver);

	return ret;
}
module_init(cy8ctma395_init);

static void __exit cy8ctma395_exit(void)
{
	serdev_device_driver_unregister(&cy8ctma395_ts_driver);
	i2c_del_driver(&cy8ctma395_i2c_driver);
}
module_exit(cy8ctma395_exit);

MODULE_AUTHOR("CyanogenMod Touchpad Project");
MODULE_AUTHOR("Herman van Hazendonk <github.com@herrie.org>");
MODULE_DESCRIPTION("Cypress CY8CTMA395 Touchscreen serdev Driver");
MODULE_LICENSE("GPL");

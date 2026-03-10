# Novacom USB Gadget Architecture Analysis

The **novacom** driver was Palm's proprietary USB communication channel used for developer/debugging access to webOS devices. It provides a vendor-specific USB interface for binary data transfer between a host computer and the device.

## Key Characteristics

- **USB Class**: Vendor Specific (`0xFF`)
- **SubClass**: `0x47`, Protocol: `0x11` (Palm-specific identifiers)
- **Transfer Type**: Bulk (2 endpoints - IN and OUT)
- **Speeds**: Full-speed (64 bytes) and High-speed (512 bytes)

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           HOST COMPUTER                                     │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    novacomd (userspace daemon)                       │   │
│  │         Communicates via libusb with vendor-specific interface       │   │
│  └──────────────────────────────┬──────────────────────────────────────┘   │
└─────────────────────────────────┼───────────────────────────────────────────┘
                                  │ USB Cable
                                  ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         webOS DEVICE (Palm/HP)                              │
│                                                                             │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │                    USB GADGET LAYER (kernel)                          │ │
│  │  ┌─────────────────────────────────────────────────────────────────┐  │ │
│  │  │              ROCKHOPPER COMPOSITE GADGET                        │  │ │
│  │  │                  (rockhopper.c)                                 │  │ │
│  │  │                                                                 │  │ │
│  │  │  Configurations:                                                │  │ │
│  │  │  ┌─────────────┬─────────────┬─────────────┬─────────────────┐  │  │ │
│  │  │  │ Config 1    │ Config 2    │ Config 5    │ Config 6        │  │  │ │
│  │  │  │ UMS only    │ UMS+NOVACOM │ RNDIS+UMS+  │ ACM+Serial+     │  │  │ │
│  │  │  │             │             │ NOVACOM     │ NOVACOM         │  │  │ │
│  │  │  └─────────────┴─────────────┴─────────────┴─────────────────┘  │  │ │
│  │  └─────────────────────────────────────────────────────────────────┘  │ │
│  │                              │                                         │ │
│  │                              ▼                                         │ │
│  │  ┌─────────────────────────────────────────────────────────────────┐  │ │
│  │  │              NOVACOM FUNCTION DRIVER (f_novacom.c)              │  │ │
│  │  │                                                                 │  │ │
│  │  │   struct f_novacom                                              │  │ │
│  │  │   ├── usb_function (bind, unbind, set_alt, disable)             │  │ │
│  │  │   ├── state: UNBOUND → CLOSED → OPENED                          │  │ │
│  │  │   ├── connect_state: DISCONNECTED ↔ CONNECTED                   │  │ │
│  │  │   │                                                             │  │ │
│  │  │   ├── ep_in (struct novacom_ep)                                 │  │ │
│  │  │   │   ├── usb_ep *ep (bulk IN)                                  │  │ │
│  │  │   │   ├── usb_request *req                                      │  │ │
│  │  │   │   ├── semaphore sem                                         │  │ │
│  │  │   │   └── wait_queue_head_t wait                                │  │ │
│  │  │   │                                                             │  │ │
│  │  │   ├── ep_out (struct novacom_ep)                                │  │ │
│  │  │   │   ├── usb_ep *ep (bulk OUT)                                 │  │ │
│  │  │   │   ├── usb_request *req                                      │  │ │
│  │  │   │   ├── semaphore sem                                         │  │ │
│  │  │   │   └── wait_queue_head_t wait                                │  │ │
│  │  │   │                                                             │  │ │
│  │  │   └── event[5] (usb_gadgetfs_event queue)                       │  │ │
│  │  │       ├── GADGETFS_CONNECT                                      │  │ │
│  │  │       ├── GADGETFS_DISCONNECT                                   │  │ │
│  │  │       ├── GADGETFS_SETUP                                        │  │ │
│  │  │       └── GADGETFS_SUSPEND                                      │  │ │
│  │  └─────────────────────────────────────────────────────────────────┘  │ │
│  │                              │                                         │ │
│  │                              ▼                                         │ │
│  │  ┌─────────────────────────────────────────────────────────────────┐  │ │
│  │  │           CHARACTER DEVICE INTERFACE (misc devices)             │  │ │
│  │  │                                                                 │  │ │
│  │  │  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐   │  │ │
│  │  │  │ /dev/novacom_ep0│ │/dev/novacom_ep_in│/dev/novacom_ep_out│   │  │ │
│  │  │  │                 │ │                 │ │                 │   │  │ │
│  │  │  │ Control/Events  │ │ Bulk IN (write) │ │ Bulk OUT (read) │   │  │ │
│  │  │  │                 │ │                 │ │                 │   │  │ │
│  │  │  │ • open()        │ │ • open()        │ │ • open()        │   │  │ │
│  │  │  │ • read() events │ │ • write() data  │ │ • read() data   │   │  │ │
│  │  │  │ • poll()        │ │ • release()     │ │ • release()     │   │  │ │
│  │  │  │ • fasync()      │ └─────────────────┘ └─────────────────┘   │  │ │
│  │  │  │ • release()     │                                           │  │ │
│  │  │  └─────────────────┘                                           │  │ │
│  │  └─────────────────────────────────────────────────────────────────┘  │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
│                              │                                              │
│                              ▼                                              │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │                    USERSPACE DAEMON (novacomd)                        │ │
│  │                                                                       │ │
│  │  Reads/writes to character devices to communicate with host           │ │
│  │  Provides services: shell access, file transfer, app install, etc.    │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          DATA FLOW DIAGRAM                                  │
│                                                                             │
│  HOST → DEVICE (Bulk OUT):                                                  │
│  ┌──────────┐    USB     ┌──────────┐   kernel   ┌──────────────────────┐  │
│  │ Host App │ ────────→ │ ep_out   │ ────────→ │ /dev/novacom_ep_out  │  │
│  │ (libusb) │   Bulk    │ endpoint │  read()   │ userspace daemon     │  │
│  └──────────┘           └──────────┘            └──────────────────────┘  │
│                                                                             │
│  DEVICE → HOST (Bulk IN):                                                   │
│  ┌──────────────────────┐  kernel  ┌──────────┐    USB     ┌──────────┐   │
│  │ /dev/novacom_ep_in   │ ───────→│ ep_in    │ ────────→ │ Host App │   │
│  │ userspace daemon     │ write() │ endpoint │   Bulk    │ (libusb) │   │
│  └──────────────────────┘         └──────────┘            └──────────┘   │
│                                                                             │
│  USB EVENTS (Control):                                                      │
│  ┌───────────────┐  set_alt()  ┌──────────────┐  read()  ┌─────────────┐  │
│  │ USB Subsystem │ ──────────→│ f_novacom    │ ───────→│novacom_ep0  │  │
│  │ (connect/disc)│  event     │ event queue  │  poll() │ userspace   │  │
│  └───────────────┘            └──────────────┘          └─────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

## State Machine

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         STATE MACHINE                                       │
│                                                                             │
│  DEVICE STATE (novacom_state):                                              │
│                                                                             │
│      ┌──────────────┐                                                       │
│      │ STATE_UNBOUND│◄──────────────────────────────────────┐               │
│      └──────┬───────┘                                       │               │
│             │ novacom_bind()                                │               │
│             ▼                                               │               │
│      ┌──────────────┐        ep0 close()        ┌───────────┴──────┐        │
│      │ STATE_CLOSED │◄──────────────────────────│ STATE_OPENED     │        │
│      └──────┬───────┘                           └─────────┬────────┘        │
│             │ ep0 open()                                  │                 │
│             └────────────────────────────────────────────►│                 │
│                                                           │ novacom_unbind()│
│                                                           └────────────────►│
│                                                                             │
│  CONNECTION STATE (connect_state):                                          │
│                                                                             │
│      ┌──────────────────┐      set_alt()       ┌────────────────────┐       │
│      │ STATE_DISCONNECTED│ ──────────────────→│ STATE_CONNECTED     │       │
│      └─────────┬─────────┘                     └──────────┬─────────┘       │
│                │                                          │                 │
│                │◄─────────────────────────────────────────┘                 │
│                         disable() / cable unplug                            │
│                                                                             │
│  ENDPOINT STATE (novacom_ep_state):                                         │
│                                                                             │
│      ┌──────────────────┐    enable()    ┌────────────────────┐             │
│      │ STATE_EP_DISABLED │ ────────────→│ STATE_EP_ENABLED   │             │
│      └─────────┬─────────┘               └──────────┬─────────┘             │
│                │◄────────────────────────────────────┘                      │
│                            disable()                                        │
└─────────────────────────────────────────────────────────────────────────────┘
```

## USB Descriptor Structure

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      USB DESCRIPTORS                                        │
│                                                                             │
│  Interface Descriptor:                                                      │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │ bInterfaceClass    = 0xFF (Vendor Specific)                          │  │
│  │ bInterfaceSubClass = 0x47 (Palm specific)                            │  │
│  │ bInterfaceProtocol = 0x11 (Palm specific)                            │  │
│  │ bNumEndpoints      = 2                                               │  │
│  │ iInterface         = "Novacom"                                       │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│  Endpoint Descriptors:                                                      │
│  ┌────────────────────────────────┐  ┌────────────────────────────────┐    │
│  │ BULK IN                        │  │ BULK OUT                       │    │
│  │ bmAttributes = BULK            │  │ bmAttributes = BULK            │    │
│  │ FS: wMaxPacketSize = 64        │  │ FS: wMaxPacketSize = 64        │    │
│  │ HS: wMaxPacketSize = 512       │  │ HS: wMaxPacketSize = 512       │    │
│  └────────────────────────────────┘  └────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Rockhopper Composite Gadget Configurations

| Config | Name | Functions |
|--------|------|-----------|
| 1 | ums | Mass Storage only |
| 2 | ums+novacom | Mass Storage + Novacom |
| 3 | passthru | ACM + Serial |
| 4 | usbnet+passthru | RNDIS + ACM + Serial |
| 5 | usbnet+ums+novacom | RNDIS + Mass Storage + Novacom |
| 6 | passthru+novacom | ACM + Serial + Novacom |

## Key Data Structures

### struct f_novacom (Primary Function Structure)

```c
struct f_novacom {
    struct usb_function     function;       // USB gadget function interface
    struct usb_composite_dev *cdev;         // Composite device reference
    int                     data_id;        // Interface ID assigned at runtime

    struct novacom_ep       ep_in;          // IN endpoint structure
    struct novacom_ep       ep_out;         // OUT endpoint structure

    struct novacom_ep_descs fs;             // Full-speed descriptors
    struct novacom_ep_descs hs;             // High-speed descriptors

    spinlock_t              lock;           // Synchronization
    enum novacom_state      state;          // Device state
    enum connect_state      connect_state;  // Connection state
    struct usb_gadgetfs_event event[5];     // Event queue
    unsigned                ev_next;        // Next event index
    struct fasync_struct    *fasync;        // Async notification
    wait_queue_head_t       ep0_wait;       // EP0 wait queue
};
```

### struct novacom_ep (Endpoint Structure)

```c
struct novacom_ep {
    struct semaphore            sem;        // Access control
    enum novacom_ep_state       state;      // Endpoint state
    struct f_novacom            *novacom;   // Back-reference
    struct usb_ep               *ep;        // USB endpoint
    struct usb_request          *req;       // USB request
    ssize_t                     status;     // Transfer status
    char                        name[16];   // Endpoint name
    struct usb_endpoint_descriptor *desc;   // Descriptor
    wait_queue_head_t           wait;       // Completion wait queue
};
```

## Key Source Files

| File | Purpose |
|------|---------|
| `drivers/usb/gadget/f_novacom.c` | Novacom function driver (1171 lines) |
| `drivers/usb/gadget/rockhopper.c` | Composite gadget integrating novacom |
| `drivers/usb/gadget/rockhopper.h` | Rockhopper header file |

## I/O Model

- **Synchronous blocking I/O** with optional non-blocking mode (`O_NONBLOCK`)
- Uses kernel completions for wait-based transfers
- Kernel buffering with `kmalloc()` for all transfers
- Support for interrupt handling with dequeue capability (1 second timeout)
- Wrong-direction I/O detection: Halts endpoint with `EBADMSG` error

## Key Functions

| Function | Purpose |
|----------|---------|
| `novacom_bind_config()` | Main entry point, registers misc devices |
| `novacom_bind()` | Allocates endpoints, copies descriptors |
| `novacom_set_alt()` | Activates interface, enables endpoints |
| `novacom_connect()` | Signals connection to userspace |
| `novacom_disconnect()` | Signals disconnection to userspace |
| `novacom_ep_io()` | Core I/O function using completions |
| `novacom_ep_read()` | Read from OUT endpoint |
| `novacom_ep_write()` | Write to IN endpoint |

## Summary

The novacom gadget was Palm's clever solution for developer access that:

1. **Bypassed traditional serial/ADB**: Used vendor-specific USB bulk endpoints for a custom binary protocol
2. **Userspace-controlled**: Exposed 3 character devices letting a userspace daemon (`novacomd`) handle the protocol
3. **Composite integration**: Worked alongside mass storage, networking (RNDIS), and serial functions via the Rockhopper composite gadget
4. **Event-driven**: Used a gadgetfs-like event queue for USB state notifications (connect/disconnect/setup)
5. **Synchronous I/O**: Implemented blocking read/write with kernel completions for reliable transfer

## License

- **f_novacom.c**: Copyright (C) 2008-2009 Palm, Inc. - GPL v2
- **rockhopper.c**: Copyright (C) 2009 Palm, Inc., Copyright (C) 2011 HP - GPL v2

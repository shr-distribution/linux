==============================
Linux USB Novacom Gadget Driver
==============================

:Copyright: |copy| 2008-2009 Palm, Inc.
:Copyright: |copy| 2024-2026 Herman van Hazendonk <github.com@herrie.org>

.. |copy| unicode:: U+000A9


General
=======

The novacom function driver implements the device-side endpoint of the
*novacom* binary transport protocol used by Palm/HP webOS devices (Palm Pre
family, HP TouchPad). The host-side daemon ``novacomd`` opens the USB
device, runs its own framing on top of two bulk pipes, and exposes virtual
TTYs / file transfer / debug shells to applications such as the ``novacom``
command-line tool.

The kernel side is intentionally thin: it presents a vendor-specific
interface to the USB host and three character devices to userspace on the
device. All framing, multiplexing, authentication and command interpretation
happen in userspace.


USB interface
=============

The function exposes a single interface with two bulk endpoints:

============================= ==========
``bInterfaceClass``           ``0xFF`` (Vendor Specific)
``bInterfaceSubClass``        ``0x47``
``bInterfaceProtocol``        ``0x11``
``bNumEndpoints``             2
``bEndpoint`` (bulk IN)       device-to-host
``bEndpoint`` (bulk OUT)      host-to-device
``wMaxPacketSize`` (FS / HS)  64 / 512
============================= ==========

SuperSpeed is not advertised: there is no tested SS-capable hardware
shipping the host-side daemon, and SS bulk needs ``bMaxBurst`` tuning that
has not been validated.


Character devices
=================

Three misc devices are created when the function is instantiated:

============================= ============================================
``/dev/novacom_ep0``          Control / event endpoint
``/dev/novacom_ep_in``        Bulk IN (device-to-host)
``/dev/novacom_ep_out``       Bulk OUT (host-to-device)
============================= ============================================

Only one function instance can be live at a time; ConfigFS will return
``-EBUSY`` on a second ``mkdir`` under ``functions/novacom.*``.


Control/event endpoint
----------------------

``/dev/novacom_ep0`` carries gadgetfs-style events. Each ``read()`` returns
one or more ``struct usb_gadgetfs_event`` records (see
``<linux/usb/gadgetfs.h>``). The buffer passed to ``read()`` must be at
least ``sizeof(struct usb_gadgetfs_event)``.

Events emitted:

``GADGETFS_SETUP``
    The host has set the device configuration. ``event.u.setup.bRequest``
    is ``USB_REQ_SET_CONFIGURATION`` and ``event.u.setup.wValue`` carries
    the configuration value (little-endian).

``GADGETFS_DISCONNECT``
    The host has dropped the connection (cable unplug, host disable, or
    UDC unbind).

``poll()`` returns ``EPOLLIN | EPOLLRDNORM`` when an event is queued.

``fcntl(F_SETOWN)`` + ``F_SETFL(O_ASYNC)`` is supported; ``SIGIO`` is
delivered on event arrival.


Bulk endpoints
--------------

``/dev/novacom_ep_in`` and ``/dev/novacom_ep_out`` accept ``read()`` and
``write()`` of arbitrary length.

* ``write(novacom_ep_in, ...)`` queues a single bulk IN transfer of the
  given length and blocks until the host reads it or the transfer is
  cancelled. Reading from ``novacom_ep_in`` issues a STALL to the host on
  the IN endpoint and returns ``-EBADMSG``.

* ``read(novacom_ep_out, ...)`` queues a single bulk OUT transfer of the
  given length and blocks until the host writes that many bytes (or sends
  a short packet, in which case the return value is the actual byte count).
  Writing to ``novacom_ep_out`` issues a STALL on the OUT endpoint and
  returns ``-EBADMSG``.

* Lengths larger than the per-transfer cap (currently 256 KiB) are
  truncated to the cap. Userspace must iterate to transfer larger payloads.

* ``O_NONBLOCK`` is honoured. A non-blocking call returns ``-EAGAIN`` if
  the endpoint is not yet enabled or another I/O is in flight on the same
  endpoint.

* A blocked I/O can be aborted with a signal; the in-flight USB request is
  dequeued and the call returns ``-EINTR``.


How to use it
=============

ConfigFS (recommended)
----------------------

::

    modprobe usb_f_novacom
    mount -t configfs none /sys/kernel/config
    mkdir /sys/kernel/config/usb_gadget/g1
    cd /sys/kernel/config/usb_gadget/g1

    echo 0x0830 > idVendor
    echo 0x8002 > idProduct
    mkdir strings/0x409
    echo HP            > strings/0x409/manufacturer
    echo "webOS Device" > strings/0x409/product

    mkdir configs/c.1
    mkdir functions/novacom.0
    ln -s functions/novacom.0 configs/c.1/

    echo <udc-name> > UDC          # e.g. ci_hdrc.0

The novacom function takes no per-instance attributes.


Legacy composite gadget
-----------------------

For environments without ConfigFS::

    modprobe g_novacom

This binds vendor ``0x0830`` / product ``0x8002`` (HP/TouchPad) and brings
up the novacom interface on the first available UDC.


Userspace
=========

The host-side daemon ``novacomd`` is the canonical consumer of this
interface. It opens the USB device by vendor/subclass/protocol triple,
claims interface 0, and runs the novacom framing layer over the bulk pipes.

A trivial loopback test can be performed with::

    cat /dev/novacom_ep_out > /tmp/data &
    cat /tmp/data > /dev/novacom_ep_in &

and a libusb tool on the host issuing matching bulk OUT / bulk IN
transactions.

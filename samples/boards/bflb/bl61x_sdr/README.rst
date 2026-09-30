.. zephyr:code-sample:: bflb-bl61x-sdr
   :name: BL61x WLAN RF I/Q capture

   Capture raw WLAN receiver samples and serve them with the ESP-SDR protocols.

Overview
********

This sample turns the BL616/BL618 2.4 GHz WLAN receiver into an SDR. The RF is calibrated by
the phyrf blob, tuned with ``phy_set_channel()`` as for a WLAN link, and the RF state machine is
then held in its receive state, which connects the front end without running the modem. The RF
SRAM dump engine writes raw RX samples into RAM; its register sequence comes from the
``sram_rx`` command of the vendor manufacturing library (``libbl616_phyrf_mfg.a``).

Two capture paths exist, each following the `ESP-SDR`_ firmware:

UART and USB CDC-ACM, snapshots
   Finite captures served with the ESP-SDR burst protocol 6, on the console UART at 2 MBaud
   and on a USB CDC-ACM port, so the `ESP-SDR web viewer`_ can be used unmodified. The device
   reports itself as ``C6SDR`` because the viewer only accepts ESP32 family identities. As on
   the ESP32 targets with two serial ports, one port controls the radio at a time: the other
   one gets ``ERR busy`` until ``RELEASE`` or five seconds without a command.

   Snapshots are offered at 80, 40, 20 and 16 MS/s (``LIMITS?``). 80 MS/s comes from the ADC
   dump tap and 40 MS/s from the decimated tap of the WLAN receiver. 20 and 16 MS/s are
   resampled from the 40 MS/s tap by polyphase FIR filters (passband 8 and 6 MHz, about
   70 dB stopband, ``src/resample.c``); a 16 MS/s snapshot uses the whole 160 KiB WRAM bank.
   The viewer always requests 16380 samples, which leaves no room for lower rates.

USB, continuous ring
   The dump engine writes continuously into a 128 KiB ring at the start of WRAM (CPU view
   0x23010000, one lap every 409.6 us at 80 MS/s). A producer thread follows
   the write position, emits 1024-sample chunks behind a guard distance, skips half a ring and
   counts drops when it falls behind, and queues IQC1/IQC8 frames. The frames are sent on a
   vendor USB interface with the ESP-SDR IQRQ/IQRS control channel and IQU1 transport
   headers (see ``src/iq_usb.h``). The ring consumer, the trigger modes (interval, gapless,
   power), the configuration apply sequence with its CFG1 reports, and the stream arming are
   ported from the ESP-SDR streaming firmware.

While a USB host owns the stream the dump window belongs to the ring, and snapshot ``CAP``,
``RXRUN``, ``FREQ`` and ``RINGPROBE`` answer ``ERR busy``. A USB configuration apply waits for
a snapshot in progress to finish sending.

USB configuration
=================

``PUT_CONFIG`` accepts the complete ESP-SDR configuration document and echoes it back with
``GET_CONFIG``. On BL61x these keys take effect: ``radio.rf_freq_hz`` (1 MHz steps),
``iq_engine.adc_source_sel`` (dump tap 0-3), ``iq_engine.adc_decimation`` (software, 1-10),
``trigger`` (interval and power modes; the AGC trigger is rejected because the dump words carry
no AGC state), and the stream format. The ESP32 radio specific keys (loopback, TX tones, dummy
Wi-Fi TX, filter overrides, expert gains, DC offset servo) and
``radio.frequency_correction_ppb`` are validated and stored only.
``TX_ARM``/``TX_COMMIT``/``TX_ABORT`` return an error. The device uses the Zephyr sample VID
0x2FE3, PID 0x4531 and the product string ``ESP-SDR``.

Status
******

Tested on a BL618 (bl618g0):

* Reception checked with a CW tone at 2413.000 MHz: 60 to 88 dB SNR at every rate, at the
  right frequency and on the right side of the spectrum in the viewer, from 2200 to 2700 MHz
  tuning. With the default crystal load trim the bl618g0 received 39 ppm high; the
  ``bl618g0`` overlay sets the trim measured on that board (within 1 ppm, about 1.4 ppm per
  step, the eFuse trim slots were empty).
* Dump words hold 11-bit signed samples at 80 MS/s (12-bit at 40 MS/s), sign-extended to 16
  bits, with I in bits 31:16 and Q in bits 15:0 in the ESP-SDR orientation. ``CAP16``/``CAP20``
  and IQC8 scale them to the 8 and 10 bits of the ESP-SDR formats; ``CAP`` and IQC1 carry the
  raw words.
* The dump end address is exclusive. The WRAM bank holds one sample per word; the other bank
  (``RF_DUMP_SRAM_WRAM`` cleared) writes every 8 bytes into OCRAM at 0x22FE0000, which is
  Zephyr RAM, so it is not used. The 0x21xxxxxx views are not accessible from the CPU.
* ``RF_DUMP_MODE`` is a continuous mode: the engine wraps over the window until ``START`` is
  cleared and counts completed passes in bits 23:16 of the control register. No fine write
  pointer register was found, so the write position is derived from time.
* The snapshot path and the web viewer protocol on both serial ports, and the USB control
  channel and IQC1/IQC8 streaming (header CRCs, sequence and drop accounting) work.
* Snapshot rates for 16380 samples at 80 MS/s: about 100 ``CAP16`` (90 ``CAP20``) per second
  on CDC-ACM with 8 KiB transfer buffers, and 5.9 ``CAP16`` (4.7 ``CAP20``) per second on the
  2 MBaud UART, which is 97 % of the line rate. The vendor stream moves about 10.9 MB/s of
  IQC1 frames or 4250 IQC8 frames per second, bound by the uncached WRAM reads.
* Chrome on macOS loses CDC-ACM data at these rates: its Web Serial backend enables
  ``PARMRK`` and keeps ``IEXTEN``, which takes the tty off the flow controlled input path, and
  bytes arriving while its buffer is full are dropped. The UART path and Firefox are not
  affected.
* ``GAIN HARDWARE`` runs a software AGC before every snapshot and while streaming; it lowers
  the gain when the samples pile up at the limit of the analog stage, below ADC full scale.

Not verified: tuning outside 2200 to 2700 MHz. The gain ladder indices are not calibrated in
dB.

UART commands
*************

Besides the ESP-SDR commands (``INFO``, ``CAPS``, ``LIMITS?``, ``RANGE?``, ``FREQ``,
``GAIN``, ``CAP16``, ``CAP20``, ``RXRUN``, ``SYNC``, ``RELEASE``, ``TRANSPORT?``), the
following commands help to reverse engineer the capture path:

``CAP <samples> <rate-index>``
   Raw 32-bit dump words, ``samples * 4`` bytes.

``DUMPSEL <0-3>``
   Dump tap select for 80 MS/s snapshots and the stream (``rx_test_sel`` in the vendor RF
   dump): 0 is the 80 MS/s ADC tap, 1 the 40 MS/s decimated tap, 2 a 16 MS/s tap that is idle
   in WLAN receive, 3 a filtered 80 MS/s tap.

``IQFMT <i-lsb> <q-lsb> <width>``
   Bit layout used to convert dump words for 80 MS/s ``CAP16``, ``CAP20`` and IQC8 frames. The
   default is ``IQFMT 16 0 11``.

``RINGPROBE``
   Fills the ring with a marker, runs the continuous dump for a few laps and answers
   ``RINGPROBE <unwritten> <rewrap-unwritten> <reg>:<first>:<last>:<changes> ...``. Zero
   unwritten words after the first wait and after re-marking 1024 words show that the engine
   wraps; the listed RF registers (offsets from 0x20001000) changed while it ran and are
   write pointer candidates.

``RINGMODE <ctrl-bits> <sram-bits>``
   Bits ORed into the dump control register (0x240) and the SRAM mode field (0x23C) for the
   continuous dump. The default is ``RINGMODE 4 0``.

``RINGPTR <offset> <lsb> <width>``
   Uses an RF register field as the ring write pointer (word index). ``RINGPTR 0 0 16`` selects
   the time-derived position.

``RINGTRACE <samples> <interval-us>``
   Starts the ring and reports ``<us> <ctrl> <status> <rewritten>`` per sample, where
   ``rewritten`` tells whether a marked ring word was written since the previous sample.

``RING?``
   Ring state: running, absolute write position, chunk index, dropped chunks.

``PEEK <addr> <words>``, ``POKE <addr> <value>``, ``FILL <addr> <words> <value>``
   Memory access (``DATA`` reply with CRC for ``PEEK``). Values may be ``0x`` prefixed.

``RFRD <offset>``, ``RFWR <offset> <value>``
   RF control block register access, offsets from 0x20001000.

Hardware bring-up
*****************

#. Check the snapshot path with ``CAP 4096 0`` and ``DUMPSEL``/``IQFMT``.
#. Run ``RINGPROBE`` and ``RINGTRACE 20 500``: the ring wraps and the lap counter in the
   control register advances once per 409.6 us.
#. Tune next to a CW source and check its position and side in the viewer, then adjust the
   crystal trim (``xtal-capcode-in``/``xtal-capcode-out`` of the ``wifi0`` node) if needed.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/bflb/bl61x_sdr
   :board: bl618g0
   :goals: build flash
   :compact:

Open the `ESP-SDR web viewer`_ in a Web Serial capable browser and connect to the USB
CDC-ACM port, or to the console UART at 2 MBaud.

.. _ESP-SDR: https://github.com/ESPARGOS/esp-sdr
.. _ESP-SDR web viewer: https://espargos.net/espsdr/app/

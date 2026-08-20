# Flight log decoder

## Capture a dump
1. PuTTY -> Session -> Logging -> All session output
2. Log file: C:\dev\capture.log, "Always overwrite it"
3. Session -> Serial, 921600 -> Open
4. Reset the board, press 'd' within 3 s
5. Wait for ---FLASH DUMP END---, then close PuTTY

## Decode
    cd C:\dev\Circuit-Testing\05_control\tools
    python decode_flight_log.py C:\dev\capture.log

## Options
    --replay            3D orientation animation
    --csv flight.csv    also write CSV
    --flight 2          decode an earlier flight (default: most recent)
    --no-plots          summary only
    --decimate 5        replay frame skip (default 10 = ~20 fps)

## Checks worth making every flight
- Tick dt should read ~5.00 ms mean
- PULSE_Y / PULSE_Z flags should never appear -- if they do,
  a servo calibration constant is wrong
- OVERRUN flags indicate the loop missed its deadline


ATSHA204A note:

watchdog:
atsha204a have a watchdog, 0.7-1.7 second enter sleep when counter not be reset
reset the counter: put the device into sleep or idle mode and then wake it up again.



io block:

send: cmd + count + opcode + param1 + param2 + data + crc

recv: count + data + crc




address counter:


nonce: produce random, and combine input-num and other info, do digest, store in tempkey

tempkey: hash of info below
32  bytes RandOut
20  bytes NumIn from input stream
1   byte Opcode (always 0x16)
1   byte Mode
1   byte LSb of Param2 (should always be 0x00)




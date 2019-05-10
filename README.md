

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



lock config:
            0x55 = The Configuration zone has write access (unlocked).
            0x00 = The Configuration zone does not have write access (locked).

data:
        when config not locked, data can be neither read nor written.
        the policies become effective upon setting the LockValue byte only.

lock value:
0x55 =  The Data and OTP zones are unlocked and has write access.

0x00 =  The Data and OTP zones are locked and take on the access policies defined in the configuration zone. 
        Slots in the Data zone can only be modified based on the corresponding WriteConfig fields. The
        OTP zone can only be modified based on the OTP mode.0x55 = The Data and OTP zones are unlocked and has write access.
        0x00 = The Data and OTP zones are locked and take on the access policies defined in the configuration
        zone. Slots in the Data zone can only be modified based on the corresponding WriteConfig fields. The
        OTP zone can only be modified based on the OTP mode.

opt: 
when opt locked, write diabled




ko debug:
#walling make error
#kernel/msm-3.18/scripts/gcc-wrapper.py
#interpret_warning(line)
#

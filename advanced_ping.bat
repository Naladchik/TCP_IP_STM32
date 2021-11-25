@echo off
FOR /L %%y IN (0, 1, 255) DO ping.exe -S 192.168.100.24 192.168.100.%%y
::ping.exe -S 192.168.100.24 192.168.100.101
pause
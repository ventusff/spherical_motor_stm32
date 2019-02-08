# Guide for logtrace
@author: ventus 

## File description

| File | description | how to work |
| --- | ---- | ----- |
| log_trace.c | Fundamental functions for log_trace. |  the log device is set via: log_trace_set_device . <br> At the beginning, the default device is console device, with only write operation allowed. <br> No control, read, etc. and the true console serial device is not configured actually. |
| log_file.c | log to DFS. | Enabled only DFS is enabled. |
| mem_log.c | log to memory. | use memlog_flush to output into console. <br> this means there are two buffers been used.(?) (while the mem log is allocated) <br> **Logs are write into the pipe(memory), and flush out via idle thread, through console device.** <br> **outbuf** is just a temp save place, in case there are to much things to flush out when memlog_flush is called. |
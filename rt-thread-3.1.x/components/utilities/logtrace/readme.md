# Guide for logtrace
@author: ventus 

## File description

| File | description | how to work |
| --- | ---- | ----- |
| log_trace.c | Fundamental functions for log_trace. |  the log device is set via: log_trace_set_device . <br> At the beginning, the default device is console device, with only write permissions. |
| log_file.c | log to DFS. | Enabled only DFS is enabled. |
| mem_log.c | log to memory. | use memlog_flush to output into console. <br> this means there are two buffers been used. <br> TODO: could add memory alloc and free. |
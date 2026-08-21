# V0.4.16 PCB 层叠与 RF 对象层分析

层叠记录数：21；RF 网络对象是否全部位于 TOP：True。

`layer_stack.csv` 来自 ASCII PCB 的 `V9_STACK_LAYER*` 字段；`rf_object_layers.csv` 统计 8 个 TX/RX 网络的对象层。这些数据可作为后续传输线/电磁仿真的输入，但尚未计算相位延迟。

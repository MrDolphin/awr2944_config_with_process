# V0.4.16 PCB 层叠与 RF 对象层分析

## 结果

输出目录：

`simulation/stages/v04_aoa_cfar_point_cloud/output/v04_16_pcb_layer_stack/`

从 ASCII PCB 的 `V9_STACK_LAYER*` 字段提取层名、LayerID、介电常数、介质厚度、材料和铜厚，并按 RF 网络统计 `Track`、`Arc`、`Pad`、`Region` 的实际层。

当前资料显示，8 个 TX/RX RF 网络对象全部位于 `TOP` 层；这可以支持“先按 TOP 层平面走线分析”的工程假设，但不等于已经知道完整 RF 参考地结构。

## 重要层叠参数

ASCII PCB 中可见的关键材料字段包括：

- `Dielectric 1`：RO3003，介电常数约 3.000，厚度 5 mil；
- `GND1`：铜层，铜厚约 1.4 mil；
- `Dielectric 2`：FR-4 High Tg，介电常数约 4.040，厚度 5 mil；
- `SIG1`：铜层，铜厚约 0.7 mil；
- `Dielectric 3`：FR-4 High Tg，介电常数约 4.040，厚度 10 mil。

完整字段以 `layer_stack.csv` 为准；未从截图或 PDF 人工猜测未出现的层。

## 对仿真的意义

RF Track/Arc 全部位于 TOP 层，说明下一步可以按 TOP 层走线做几何长度和层分类；但 77 GHz 的实际传播常数还需要微带/带状线截面、参考地、阻焊层、铜粗糙度和过渡结构。当前不把 `εr` 直接当成有效介电常数，也不把几何长度直接换算成相位补偿。

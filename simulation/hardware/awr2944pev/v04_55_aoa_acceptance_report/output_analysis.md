# V0.4.55 AoA 综合验收分析

## 结论

- 合成多场景 AoA 回归：通过；
- 真实硬件 AoA：未就绪。

## 判定依据

多场景最佳候选为 RX=`0,1,2,3`、TX=`0,1,2,3`，综合 RMSE 为 0°，最大方位/俯仰误差均为 0°。但输入来源为 synthetic known-angle fixture，校准状态为 synthetic unity not measured，且 channel_order_hardware_verified=false。

## 汇报建议

当前可以汇报“软件算法链路和多场景排列回归通过”，不能汇报“2944P 实测 AoA 已完成”或“真实角度误差为 0°”。真实硬件门禁等待 DCA1000 已知角度采集和测量 provenance。

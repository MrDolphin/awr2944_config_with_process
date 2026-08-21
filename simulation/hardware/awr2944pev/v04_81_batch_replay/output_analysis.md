# V0.4.81 多文件原始 IQ 批量回放

支持 `.bin`、`.h5`、`.npz`。每个输入文件生成独立子目录、HDF5 回放结果、契约验证状态和 output_analysis.md；`manifest.csv` 汇总全部文件。

`.bin` 文件必须额外提供 chirp 数和每 chirp sample 数。真实采集数据仍需确认 LVDS wire order、TDM TX 序列、通道顺序和 TI 校准状态。

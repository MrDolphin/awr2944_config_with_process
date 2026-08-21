# V0.4.15 RF 网络几何连通性

识别对象：{'Arc': 26, 'Pad': 8, 'Track': 31, 'Region': 8}。每条 RF 网络使用 1 mil 节点量化容差建立几何图。

`pad_to_region_connected` 只表示 PCB ASCII 几何对象在容差内可连通，不表示 RF 电气连续性、阻抗匹配或相位中心。`shortest_pad_to_region_mm` 是 Track/Arc/Region 的几何路径长度，不能直接换算成 77 GHz 相位延迟。

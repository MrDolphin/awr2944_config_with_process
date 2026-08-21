# AWR2944P PCB/CAD 资料包审计

本文件由 `simulation/inspect_pcb_package.py` 生成。哈希用于确认后续分析引用的是同一份资料。

## 文件清单

| 文件 | 字节数 | 类型 | 可提取内容 | SHA-256 |
|---|---:|---|---|---|
| `C:\Users\56461\Downloads\2944p资料\sprr440a (1)\SPRR440\3D_PCB\PROC113D_BRD.step` | 31928689 | step_mechanical_cad | 可提取机械实体/包络；通常没有 RF 相位中心 | `4f94d0b39d451f2c6599faf4530dece865e8efc15bc088a384c2fa27ada0f271` |
| `C:\Users\56461\Downloads\2944p资料\sprr440a (1)\SPRR440\Assembly Drawings\PROC113D(001)_Assy.PDF` | 2625116 | pdf_release | 用于装配方向、层叠、标注和人工复核 | `313499bd053476b3e6e9767f701f9030e354410d5c0ef326714d1955ba5a714b` |
| `C:\Users\56461\Downloads\2944p资料\sprr440a (1)\SPRR440\BOM\PROC113D(001)_BOM.xls` | 93184 | bom | 用于器件/版本追溯，不提供天线相位中心 | `a196204835e14a064206db46d182907d1e1c25a0ef8d0eddd716a585dd030c3b` |
| `C:\Users\56461\Downloads\2944p资料\sprr440a (1)\SPRR440\Schematics\PROC113D(001)_Sch.PDF` | 4542466 | pdf_release | 用于装配方向、层叠、标注和人工复核 | `1391965069217a2fa1bbdd4d53d57a4bd7715b7edfb8e8db7c047432b29239cf` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_3V3_Supply_Reference.SchDoc` | 297472 | altium_project | 用于设计版本和连接关系追溯 | `9ca916a3e058fbb98fab9e7ac718610d38011de47f1bc89fc6e807dd2ee47515` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_ASCII.PcbDoc` | 44581965 | altium_ascii_pcb | 可解析铜区/网络/层；可做几何中心近似 | `eba1791d75a42fe02ad2aa9672398f9252721cc243366510d27e3b170a9ceda6` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_AssemblyRelease.OutJob` | 18887 | altium_project | 用于设计版本和连接关系追溯 | `c439c8a8a5cfa87cd0cb775ce85ec1902bc400c6f63d3b345c9c831ad3c9408f` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_Auto_Ethernet_conn.SchDoc` | 281088 | altium_project | 用于设计版本和连接关系追溯 | `148e46c4fe5bbc763287b6f56759a82bf816c763be1305172af878fa6077b720` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_Auto_Ethernet_PHY.SchDoc` | 950784 | altium_project | 用于设计版本和连接关系追溯 | `cfcbb9225636cd0b83f841401d25a4b6b6ac025d1f92094f6c5522d658999c10` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_BRD.PcbDoc` | 23074816 | altium_ole_pcb | 需 Altium 导出 ASCII 或专用 OLE 解析器；当前不直接解析 | `acef802923aa30800880c632d8046a093828a73976f84e35c804b530d68a9638` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_BRD.PcbDoc.htm` | 4973 | other | 未定义自动提取规则 | `8072ee4ff312782b7771582c152f4dbd3c3f5daa174fc82e0d9310dc55924a02` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_CAN_Interface.SchDoc` | 468480 | altium_project | 用于设计版本和连接关系追溯 | `8cdaba0c37b789f954dbd47310ba095c380391e7fdfd5386a3ed561a2903755a` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_Common Documentation.OutJob` | 102581 | altium_project | 用于设计版本和连接关系追溯 | `47a188a4f31502ccf2be3f822e37ee8546f8e5dd033ea7ff500f8fc54e019252` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_CoverSheet.SchDoc` | 337408 | altium_project | 用于设计版本和连接关系追溯 | `3ca6a129f571a20edc5c19dbf13c9c892a99605c47908ab7ebcabd800d694061` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_Current_Sensors.SchDoc` | 391168 | altium_project | 用于设计版本和连接关系追溯 | `1870caa7e8f6079f23868645c62854b659fa567cb4a477481d4c0e89aabca0b6` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_Debug_Connector.SchDoc` | 718848 | altium_project | 用于设计版本和连接关系追溯 | `081988e482be6e6f939b8425ceb6a94250c2345f2d962da2ea74d0087913cf45` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_Decoupling_Reference.SchDoc` | 504832 | altium_project | 用于设计版本和连接关系追溯 | `c24ca177fa157981a45d70f2a0a5f0cf8a22cc80e068752681698988e88020f7` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_Ethernet_Magnetics.SchDoc` | 215552 | altium_project | 用于设计版本和连接关系追溯 | `11d2a88188bca32b6c00456594a5f8a3eb033dff150d2d43d404e14eac1bafa5` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_Ethernet_PHY.SchDoc` | 686080 | altium_project | 用于设计版本和连接关系追溯 | `cea48d9394716a5d006817c78eaf4b2c15a64f6c77fbda0010b2cff2f65782e1` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_Ethernet_PWR.SchDoc` | 407040 | altium_project | 用于设计版本和连接关系追溯 | `b6e4a3cfe74ad3c27cc725582584940d2795737373f832ee8f2a1c1b07dab1d7` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_FabricationRelease.OutJob` | 44790 | altium_project | 用于设计版本和连接关系追溯 | `f72f209df959801895e018432117ce38c962e33191d43fadb35795e53c0c90eb` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_FTDI.SchDoc` | 701952 | altium_project | 用于设计版本和连接关系追溯 | `4113e9297051a27e1ad17da3e174514137abb76f1c47048c10857023354f044b` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_FTDI_PWR.SchDoc` | 268288 | altium_project | 用于设计版本和连接关系追溯 | `e132e6399d7d3ff3d51617bc899eb0d1f003e69481e3bd6093ff0e6af286ea4d` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_Hardware.SchDoc` | 458752 | altium_project | 用于设计版本和连接关系追溯 | `c0a483a16d52385833adea13cc9e6c44ea472a4e7537f011a8632d435cef90db` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_IO_Reference.SchDoc` | 1050112 | altium_project | 用于设计版本和连接关系追溯 | `d7a238a99ba386ab7337277a9e4f19149e0bdae455853bb847b05a12a34891f6` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_JTAG_EMU_Connector.SchDoc` | 479744 | altium_project | 用于设计版本和连接关系追溯 | `8469ba770dd18c28b113f01af837e95b00afdb4dc933ac26025348fbde432d9e` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_Other BOM Reports.OutJob` | 12747 | altium_project | 用于设计版本和连接关系追溯 | `275b387c5e664798478c0ac3302f2ac5d98625f2ddc9c1b768043d231c695d93` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_PCBlayers.PDF` | 18625624 | pdf_release | 用于装配方向、层叠、标注和人工复核 | `f30d8909cdcc8755dc003fb96a89c9a86694aa15bc09b4c9f3d0836083e183d0` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_PMIC_Reference.SchDoc` | 997376 | altium_project | 用于设计版本和连接关系追溯 | `224a73a6c6623a3237f34e9f45ed9cbcd12b608b7293f7a5a0dcf3d3e42f188d` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_PWR_Reference.SchDoc` | 487424 | altium_project | 用于设计版本和连接关系追溯 | `2440a00c972526d9c0539138c4519babf204700a5356479adea83d14704f10ef` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_PWR_RST_LED.SchDoc` | 453632 | altium_project | 用于设计版本和连接关系追溯 | `4689d75ad3a2ffcd51258d2be2fdc8383ad116ba7bbad1725d725e8081de5987` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_QSPI_Flash_Reference.SchDoc` | 221184 | altium_project | 用于设计版本和连接关系追溯 | `8bf45335ebc07a5c37518dee88820b586a253563d0ba4d02a296e23154a2f9ab` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_SOP_Reference.SchDoc` | 284160 | altium_project | 用于设计版本和连接关系追溯 | `952cbe5cd56bd733598d232ca9879a47e44d62dc8109263d33540c00d598e047` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_Temp_Sensor.SchDoc` | 117760 | altium_project | 用于设计版本和连接关系追溯 | `ecb62b0548644130a2cb4972106731288275716574a24407b1531c08453ef385` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_Validation.OutJob` | 9419 | altium_project | 用于设计版本和连接关系追溯 | `f2919616aa6db71c2e6d44d664dd96fdc8c08411d454c1bd3f9e604979b3d818` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_VPP_LDO.SchDoc` | 118272 | altium_project | 用于设计版本和连接关系追溯 | `2c698fe206c0ff817e7a229a535fa68b15d200fa62dd35c30b9797cde9a7ad7d` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_XDS110 Interface_1A.SchDoc` | 249856 | altium_project | 用于设计版本和连接关系追溯 | `a0a2b2c301f4b6b505a6986ace088b6bfbc6593eef5756381bdb30cd3fe6c4e9` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_XDS110 Interface_1B.SchDoc` | 588288 | altium_project | 用于设计版本和连接关系追溯 | `0b4d076dd5c123a970aa9a790a919c9e1a7078a25a6769b2e9c07e4f66789547` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_xWR2944EVM.Dat` | 75732 | altium_project | 用于设计版本和连接关系追溯 | `763c90176e6556cdfa3a49fd4d5986391394d661681bb4c129954a1988827875` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_xWR2944EVM.OutJob` | 4189 | altium_project | 用于设计版本和连接关系追溯 | `780e27e63e4cedf4cd85616c29745ae37ac4e5ee5b8fe66eb796e6817ec30eaf` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_xWR2944EVM.PrjPcb` | 117682 | altium_project | 用于设计版本和连接关系追溯 | `a8a139b48288febd86f7cd57a418116acba6616c2e73b348fe4047816ea2ea22` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_xWR2944EVM.PrjPcbStructure` | 71 | other | 未定义自动提取规则 | `b21ca09fc83ab30866b3c85ead6bf68dc9f246f6167477bf11028c1302bed9d3` |
| `C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_xWR2944EVM.PrjPcbVariants` | 4096 | other | 未定义自动提取规则 | `4255f20e827dda3f9662222af007314d367dd6b43c2f19dd34e3b81a7d7e99e7` |

## 当前判断

- `PROC113D_ASCII.PcbDoc` 是当前最有价值的自动化输入：已能按 RF 网络提取 8 个 TX/RX 铜区的顶点、包围盒和几何中心。
- `PROC113D_BRD.PcbDoc` 是 Altium OLE 二进制文件。它可能包含更完整的层和对象信息，但在未导出 ASCII 或未使用 Altium 官方解析链之前，不能声称已经提取成功。
- `PROC113D_BRD.step` 适合验证板框、安装方向和机械包络，不足以单独给出天线电气相位中心。
- 装配图、层叠图和原理图可用于确认 TX/RX 区域的朝向、层号、馈电关系；BOM 只做版本追溯。
- 所有由 PCB 铜区几何中心得到的阵列坐标都必须标记为 `not_electrical_phase_center`，进入 AoA 前还需要仿真或角反射器实测校准。

## 进入 AoA 管线所需的最小数据

1. 每个 TX/RX 天线的唯一编号、网络名、层号和馈电点坐标；
2. PCB 坐标原点、单位、X/Y 轴方向以及板面朝向；
3. 从 TX/RX 编号到 CFG `Tx0Rx0...Tx3Rx3` 的映射；
4. 真实天线相位中心或经过角反射器标定得到的等效坐标；
5. 与坐标对应的校准幅相矩阵和采集 CFG。

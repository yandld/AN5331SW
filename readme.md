NXP 应用笔记AN5331： 利用FRDM-K64制作离线烧写器 AN源代码

![](frdm_k64.png)

支持LPC55S69

对应应用笔记： https://www.nxp.com/docs/en/application-note/AN5331.pdf



KE02 版本支持 输出OK和 错误信号：



OK信号： PTC11(J4-10)  烧录器复位后为低电平，当模块成功烧录芯片后，变为高电平并保持

ERR信号: PTC10(J4-12)  烧录器复位后为低电平，当模块烧录错误后，变为高电平并保持


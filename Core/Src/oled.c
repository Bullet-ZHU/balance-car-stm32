#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "main.h"
#include <stdint.h>

// OLED软件显存 128列×8页(64行)，多开一点防止越界
uint8_t OLED_GRAM[144][8];

/**
 * @brief  设置OLED显示反色/正常
 * @param  i:0 正常显示(黑底白字)  1 反色显示(白底黑字)
 * @retval 无
 */
void OLED_ColorTurn(uint8_t i)
{
	if(i==0)
	{
		OLED_WR_Byte(0xA6,OLED_CMD);//正常显示
	}
	if(i==1)
	{
		OLED_WR_Byte(0xA7,OLED_CMD);//反色显示
	}
}

/**
 * @brief  屏幕180度旋转显示设置
 * @param  i:0 不旋转  1 旋转180°
 * @retval 无
 */
void OLED_DisplayTurn(uint8_t i)
{
	if(i==0)
	{
		OLED_WR_Byte(0xC8,OLED_CMD);//不旋转
		OLED_WR_Byte(0xA1,OLED_CMD);
	}
	if(i==1)
	{
		OLED_WR_Byte(0xC0,OLED_CMD);//180度旋转
		OLED_WR_Byte(0xA0,OLED_CMD);
	}
}

/**
 * @brief  IIC简单延时，用于I2C时序
 * @retval 无
 */
void IIC_delay(void)
{
	uint8_t t=3;
	while(t--);
}

/**
 * @brief  I2C起始信号
 * @retval 无
 */
void I2C_Start(void)
{
	OLED_SDA_Set();
	OLED_SCL_Set();
	IIC_delay();
	OLED_SDA_Clr();
	IIC_delay();
	OLED_SCL_Clr();
	IIC_delay();
}

/**
 * @brief  I2C停止信号
 * @retval 无
 */
void I2C_Stop(void)
{
	OLED_SDA_Clr();
	OLED_SCL_Set();
	IIC_delay();
	OLED_SDA_Set();
}

/**
 * @brief  I2C等待应答信号
 * @retval 无
 */
void I2C_WaitAck(void)
{
	OLED_SDA_Set();
	IIC_delay();
	OLED_SCL_Set();
	IIC_delay();
	OLED_SCL_Clr();
	IIC_delay();
}

/**
 * @brief  I2C发送一个字节
 * @param  dat：要发送的字节数据
 * @retval 无
 */
void Send_Byte(uint8_t dat)
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		if(dat&0x80)//取出最高位输出到SDA
		{
			OLED_SDA_Set();
    }
		else
		{
			OLED_SDA_Clr();
    }
		IIC_delay();
		OLED_SCL_Set();
		IIC_delay();
		OLED_SCL_Clr();//SCL拉低，准备下一位
		dat<<=1;
  }
}

/**
 * @brief  向OLED写1字节
 * @param  dat：要写入的数据
 * @param  mode：0=命令  1=数据
 * @retval 无
 */
void OLED_WR_Byte(uint8_t dat,uint8_t mode)
{
	I2C_Start();
	Send_Byte(0x78);	//OLED I2C从机地址
	I2C_WaitAck();
	if(mode){Send_Byte(0x40);}	//写数据标志
  else{Send_Byte(0x00);}		//写命令标志
	I2C_WaitAck();
	Send_Byte(dat);
	I2C_WaitAck();
	I2C_Stop();
}

/**
 * @brief  打开OLED显示
 * @retval 无
 */
void OLED_DisPlay_On(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);//电荷泵设置命令
	OLED_WR_Byte(0x14,OLED_CMD);//开启电荷泵
	OLED_WR_Byte(0xAF,OLED_CMD);//开启屏幕显示
}

/**
 * @brief  关闭OLED显示
 * @retval 无
 */
void OLED_DisPlay_Off(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);//电荷泵设置命令
	OLED_WR_Byte(0x10,OLED_CMD);//关闭电荷泵
	OLED_WR_Byte(0xAE,OLED_CMD);//关闭屏幕显示
}

/**
 * @brief  将软件显存刷新到OLED硬件屏幕
 * @note   绘图函数只修改GRAM，必须调用本函数屏幕才会更新
 * @retval 无
 */
void OLED_Refresh(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte(0xb0+i,OLED_CMD); //设置页地址
		OLED_WR_Byte(0x00,OLED_CMD);   //设置列低地址
		OLED_WR_Byte(0x10,OLED_CMD);   //设置列高地址
		I2C_Start();
		Send_Byte(0x78);
		I2C_WaitAck();
		Send_Byte(0x40);
		I2C_WaitAck();
		for(n=0;n<128;n++)
		{
			Send_Byte(OLED_GRAM[n][i]);
			I2C_WaitAck();
		}
		I2C_Stop();
  }
}

/**
 * @brief  清屏，把软件显存全部清零并刷新屏幕
 * @retval 无
 */
void OLED_Clear(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
	   for(n=0;n<128;n++)
			{
			 OLED_GRAM[n][i]=0;//显存全部置0
			}
  }
	OLED_Refresh();//刷新显示
}

/**
 * @brief  在显存上画点
 * @param  x:点横坐标 0~127
 * @param  y:点纵坐标 0~63
 * @param  t:1=点亮该点  0=熄灭该点
 * @note   仅修改显存，需要OLED_Refresh()才显示
 * @retval 无
 */
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t)
{
	uint8_t i,m,n;
	i=y/8;		//计算属于哪一页
	m=y%8;		//页内bit位
	n=1<<m;
	if(t){OLED_GRAM[x][i]|=n;}
	else
	{
		OLED_GRAM[x][i]=~OLED_GRAM[x][i];
		OLED_GRAM[x][i]|=n;
		OLED_GRAM[x][i]=~OLED_GRAM[x][i];
	}
}

/**
 * @brief  画直线（Bresenham算法）
 * @param  x1,y1：起点坐标
 * @param  x2,y2：终点坐标
 * @param  mode：1画线，0擦除线
 * @note   仅修改显存，需要OLED_Refresh()才显示
 * @retval 无
 */
void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t mode)
{
	uint16_t t;
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //x方向差值
	delta_y=y2-y1;
	uRow=x1;//起点x
	uCol=y1;//起点y
	if(delta_x>0)incx=1;
	else if (delta_x==0)incx=0;//垂直线
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线
	else {incy=-1;delta_y=-delta_x;}
	if(delta_x>delta_y)distance=delta_x; //取最大步长
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		OLED_DrawPoint(uRow,uCol,mode);//画点
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}

/**
 * @brief  画圆（Bresenham圆算法）
 * @param  x,y：圆心坐标
 * @param  r：圆半径
 * @note   仅修改显存，需要OLED_Refresh()才显示
 * @retval 无
 */
void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r)
{
	int a, b,num;
    a = 0;
    b = r;
    while(2 * b * b >= r * r)
    {
        OLED_DrawPoint(x + a, y - b,1);
        OLED_DrawPoint(x - a, y - b,1);
        OLED_DrawPoint(x - a, y + b,1);
        OLED_DrawPoint(x + a, y + b,1);

        OLED_DrawPoint(x + b, y + a,1);
        OLED_DrawPoint(x + b, y - a,1);
        OLED_DrawPoint(x - b, y - a,1);
        OLED_DrawPoint(x - b, y + a,1);

        a++;
        num = (a * a + b * b) - r*r;
        if(num > 0)
        {
            b--;
            a--;
        }
    }
}

/**
 * @brief  显示单个英文字符
 * @param  x,y：字符左上角坐标
 * @param  chr：要显示的ASCII字符
 * @param  size1：字号 8/12/16/24
 * @param  mode：0正常显示，1反色显示
 * @note   仅写显存，调用OLED_Refresh刷新
 * @retval 无
 */
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size1,uint8_t mode)
{
	uint8_t i,m,temp,size2,chr1;
	uint8_t x0=x,y0=y;
	if(size1==8)size2=6;
	else size2=(size1/8+((size1%8)?1:0))*(size1/2);
	chr1=chr-' ';  //转换为字库下标
	for(i=0;i<size2;i++)
	{
		if(size1==8)
			  {temp=asc2_0806[chr1][i];}
		else if(size1==12)
        {temp=asc2_1206[chr1][i];}
		else if(size1==16)
        {temp=asc2_1608[chr1][i];}
		else if(size1==24)
        {temp=asc2_2412[chr1][i];}
		else return;
		for(m=0;m<8;m++)
		{
			if(temp&0x01)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp>>=1;
			y++;
		}
		x++;
		if((size1!=8)&&((x-x0)==size1/2))
		{x=x0;y0=y0+8;}
		y=y0;
  }
}

/**
 * @brief  显示ASCII字符串
 * @param  x,y：字符串起始坐标
 * @param  *chr：字符串指针
 * @param  size1：字号 8/12/16/24
 * @param  mode：0正常，1反色
 * @note   仅写显存，调用OLED_Refresh刷新
 * @retval 无
 */
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t size1,uint8_t mode)
{
	while((*chr>=' ')&&(*chr<='~'))//可打印ASCII范围
	{
		OLED_ShowChar(x,y,*chr,size1,mode);
		if(size1==8)x+=6;
		else x+=size1/2;
		chr++;
  }
}

/**
 * @brief  幂运算，内部工具函数 m^n
 * @param  m：底数
 * @param  n：指数
 * @retval 运算结果
 */
uint32_t OLED_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;
	while(n--)
	{
	  result*=m;
	}
	return result;
}

/**
 * @brief  显示数字
 * @param  x,y：起始坐标
 * @param  num：要显示数字
 * @param  len：显示几位，高位补0
 * @param  size1：字号
 * @param  mode：0正常，1反色
 * @note   仅写显存，调用OLED_Refresh刷新
 * @retval 无
 */
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1,uint8_t mode)
{
	uint8_t t,temp,m=0;
	if(size1==8)m=2;
	for(t=0;t<len;t++)
	{
		temp=(num/OLED_Pow(10,len-t-1))%10;
			if(temp==0)
			{
				OLED_ShowChar(x+(size1/2+m)*t,y,'0',size1,mode);
      }
			else
			{
			  OLED_ShowChar(x+(size1/2+m)*t,y,temp+'0',size1,mode);
			}
  }
}

/**
 * @brief  显示浮点数 (带负号和小数)
 * @param  x,y: 显示位置左上角坐标 (像素)
 * @param  val: 要显示的浮点数, 支持负数 (自动加 '-' 号)
 * @param  decimals: 小数位数 (0~4), 如 1 显示 "12.3", 2 显示 "12.34"
 * @param  len: 总显示字符宽度(字符数), 格式化结果不足 len 位时右侧补空格,
 *             用于清除上一帧较长数字留下的残留字符 (如 -180.0 -> 5.2)
 * @param  size1: 字号 8/16/24
 * @param  mode: 0=反色显示 1=正常显示
 * @note   只写显存, 屏幕更新需再调用 OLED_Refresh()
 * @note   支持范围: 整数部分最多 4 位 (±9999), 内部缓冲 16 字节, len 最大取 14
 * @retval 无
 */
void OLED_ShowFloat(uint8_t x,uint8_t y,float val,uint8_t decimals,uint8_t len,uint8_t size1,uint8_t mode)
{
	uint8_t buf[16],n=0;   /* 格式化结果缓冲, n 为当前写入位置 */
	uint8_t i,tmp[8],t=0;  /* 循环变量, tmp 为数字倒序转换的临时缓冲 */
	float scale=1.0f;      /* 小数位对应的放大倍数 10^decimals */
	uint32_t scaled,int_part,frac_part;

	/* 1. 负号处理: 负数先写 '-' 再取绝对值, 后续统一按正数格式化 */
	if(val<0.0f)
	{
		buf[n++]='-';
		val=-val;
	}

	/* 2. 放大并四舍五入: 如 val=12.34, decimals=1 -> scaled=124 */
	for(i=0;i<decimals;i++) scale*=10.0f;
	scaled=(uint32_t)(val*scale+0.5f);
	int_part=scaled/(uint32_t)scale;   /* 整数部分 */
	frac_part=scaled%(uint32_t)scale;  /* 小数部分(放大后的余数) */

	/* 3. 整数部分: 逐位取模转成字符, 先倒序存 tmp, 再正序写入 buf */
	if(int_part==0)
	{
		tmp[t++]='0';
	}
	else
	{
		while(int_part>0)
		{
			tmp[t++]=(uint8_t)('0'+(int_part%10u));
			int_part/=10u;
		}
	}
	while(t>0) buf[n++]=tmp[--t];

	/* 4. 小数点与小数部分: 不足 decimals 位前面补 '0' (如 12.05 的 "05") */
	if(decimals>0)
	{
		buf[n++]='.';
		t=0;
		if(frac_part==0)
		{
			tmp[t++]='0';
		}
		else
		{
			while(frac_part>0)
			{
				tmp[t++]=(uint8_t)('0'+(frac_part%10u));
				frac_part/=10u;
			}
		}
		while(t<decimals) tmp[t++]='0';  /* 补前导 0 到 decimals 位 */
		while(t>0) buf[n++]=tmp[--t];
	}

	/* 5. 右侧补空格到 len 位: 清除上一帧较长数字的残留字符 */
	while(n<len) buf[n++]=' ';
	buf[n]='\0';

	/* 6. 交给字符串显示函数输出 */
	OLED_ShowString(x,y,buf,size1,mode);
}

/**
 * @brief  显示单个汉字
 * @param  x,y：汉字左上角坐标
 * @param  num：汉字在字库中的索引编号
 * @param  size1：字号 16/24/32/64
 * @param  mode：0正常，1反色
 * @note   需要oledfont.h提供HZK字库；仅写显存，需刷新
 * @retval 无
 */
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1,uint8_t mode)
{
	uint8_t m,temp;
	uint8_t x0=x,y0=y;
	uint16_t i,size3=(size1/8+((size1%8)?1:0))*size1;
	for(i=0;i<size3;i++)
	{
		if(size1==16)
				{temp=Hzk1[num][i];}
		else if(size1==24)
				{temp=Hzk2[num][i];}
		else if(size1==32)
				{temp=Hzk3[num][i];}
		else if(size1==64)
				{temp=Hzk4[num][i];}
		else return;
		for(m=0;m<8;m++)
		{
			if(temp&0x01)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp>>=1;
			y++;
		}
		x++;
		if((x-x0)==size1)
		{x=x0;y0=y0+8;}
		y=y0;
	}
}

/**
 * @brief  汉字滚动显示
 * @param  num：汉字总个数
 * @param  space：字之间间隔
 * @param  mode：0正常，1反色
 * @note  内部while(1)死循环，不可直接放在主循环中
 * @retval 无
 */
void OLED_ScrollDisplay(uint8_t num,uint8_t space,uint8_t mode)
{
	uint8_t i,n,t=0,m=0,r;
	while(1)
	{
		if(m==0)
		{
	    OLED_ShowChinese(128,24,t,16,mode);
			t++;
		}
		if(t==num)
			{
				for(r=0;r<16*space;r++)
				 {
					for(i=1;i<144;i++)
						{
							for(n=0;n<8;n++)
							{
								OLED_GRAM[i-1][n]=OLED_GRAM[i][n];
							}
						}
           OLED_Refresh();
				 }
        t=0;
      }
		m++;
		if(m==16){m=0;}
		for(i=1;i<144;i++)
		{
			for(n=0;n<8;n++)
			{
				OLED_GRAM[i-1][n]=OLED_GRAM[i][n];
			}
		}
		OLED_Refresh();
	}
}

/**
 * @brief  显示BMP图片
 * @param  x,y：图片左上角坐标
 * @param  sizex：图片宽度
 * @param  sizey：图片高度
 * @param  BMP[]：取模后的图片数组
 * @param  mode：0正常，1反色
 * @note   取模方式：纵向取模，字节倒序；仅写显存，需刷新
 * @retval 无
 */
void OLED_ShowPicture(uint8_t x,uint8_t y,uint8_t sizex,uint8_t sizey,uint8_t BMP[],uint8_t mode)
{
	uint16_t j=0;
	uint8_t i,n,temp,m;
	uint8_t x0=x,y0=y;
	sizey=sizey/8+((sizey%8)?1:0);
	for(n=0;n<sizey;n++)
	{
		 for(i=0;i<sizex;i++)
		 {
				temp=BMP[j];
				j++;
				for(m=0;m<8;m++)
				{
					if(temp&0x01)OLED_DrawPoint(x,y,mode);
					else OLED_DrawPoint(x,y,!mode);
					temp>>=1;
					y++;
				}
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y0=y0+8;
				}
				y=y0;
     }
	 }
}

/**
 * @brief  OLED初始化，硬件复位+SSD1306寄存器配置
 * @retval 无
 */
void OLED_Init(void)
{
	OLED_RES_Clr();
	HAL_Delay(200);
	OLED_RES_Set();

	OLED_WR_Byte(0xAE,OLED_CMD);//关闭屏幕
	OLED_WR_Byte(0x00,OLED_CMD);//设置列地址低4位
	OLED_WR_Byte(0x10,OLED_CMD);//设置列地址高4位
	OLED_WR_Byte(0x40,OLED_CMD);//设置显示起始行
	OLED_WR_Byte(0x81,OLED_CMD);//对比度设置
	OLED_WR_Byte(0xCF,OLED_CMD);//亮度值
	OLED_WR_Byte(0xA1,OLED_CMD);//列映射
	OLED_WR_Byte(0xC8,OLED_CMD);//行扫描方向
	OLED_WR_Byte(0xA6,OLED_CMD);//正常显示模式
	OLED_WR_Byte(0xA8,OLED_CMD);//多路复用设置
	OLED_WR_Byte(0x3f,OLED_CMD);//1/64占空比
	OLED_WR_Byte(0xD3,OLED_CMD);//显示偏移
	OLED_WR_Byte(0x00,OLED_CMD);//无偏移
	OLED_WR_Byte(0xd5,OLED_CMD);//时钟分频
	OLED_WR_Byte(0x80,OLED_CMD);
	OLED_WR_Byte(0xD9,OLED_CMD);//预充电周期
	OLED_WR_Byte(0xF1,OLED_CMD);
	OLED_WR_Byte(0xDA,OLED_CMD);//COM引脚硬件配置
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD);//VCOMH电压
	OLED_WR_Byte(0x30,OLED_CMD);
	OLED_WR_Byte(0x20,OLED_CMD);//地址模式
	OLED_WR_Byte(0x02,OLED_CMD);//页地址模式
	OLED_WR_Byte(0x8D,OLED_CMD);//电荷泵
	OLED_WR_Byte(0x14,OLED_CMD);//开启电荷泵
	OLED_Clear();
	OLED_WR_Byte(0xAF,OLED_CMD);//打开OLED屏幕
}

/**
 * @brief  全屏点亮，显存全部置0xFF，刷新屏幕全白
 * @retval 无
 */
void OLED_Fill(void)
{
	uint8_t i, n;
	for(i = 0; i < 8; i++)
	{
		for(n = 0; n < 128; n++)
		{
			OLED_GRAM[n][i] = 0xFF; //显存全部置1
		}
	}
	OLED_Refresh(); //刷新显存到屏幕
}


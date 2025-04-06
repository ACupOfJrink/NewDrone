import streamlit as st
import numpy as np
from PIL import Image
import base64
from io import BytesIO
# 设置网页信息  page_icons用于显示表情
st.set_page_config(page_title="空中使者——基于优化理论的无人机编队控制方法", page_icon=":drone:", layout="wide")

#函数区
def image_to_base64(image):
    buffered = BytesIO()
    image.save(buffered, format="PNG")
    return base64.b64encode(buffered.getvalue()).decode()

#CSS样式
#加粗
custom_style="""
<style>
    .Font6Bold {
        font-weight:bold;
        font size:150px;
    }
</style>
"""

#楷体
st.markdown(
    """
    <style>
    .kai-font {
        font-family: "KaiTi", "楷体", serif;
    }
    </style>
    """,
    unsafe_allow_html=True
)

#居中
st.markdown(
    """
    <style>
    .center {
        text-align: center;
    }
    </style>
    """,
    unsafe_allow_html=True
)

#楷体居中
st.markdown(
    """
    <style>
    .kai-font_center {
        font-family: "KaiTi", "楷体", serif;
        text-align: center;
    }
    </style>
    """,
    unsafe_allow_html=True
)

# 侧边栏
with st.sidebar:
    LocPri = st.sidebar.radio(
        "导航",
        (
            "主页",
            "STEP1——二维无源定位",
            "STEP2——三维无源定位",
            "STEP3——求解准确位置",
            "STEP4——纠正错误位置",
        ),
    )

#initilize load


# 定位维度
if LocPri == "主页":
    st.title('_:blue[空中使者]_——基于优化理论的无人机无源定位方法')
    st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/PositionSummary.png")) + '" width="1200"></div>', unsafe_allow_html=True)
    st.header('简介:')
    st.markdown('''
                :blue[对于二维无源定位：]此情形下，本文基于分布式控制策略，通过建立被动接收信号无人机的定位模型，从而进行自适应调节无人机位置。
                通过对4种情形进行研究，确定了二维下仅需三架信号无人机可纠正其余无人机坐标位置。
                在对位置无人机关于已有的三架信号无人机的相对位置关系，
                分类为9种子情形并进行讨论，通过几何原理得出待定位坐标的表达式。
                紧接着基于几何原理将问题转换为求解线性方程组的问题，利用迭代法进行了迭代求解，并进行了模拟仿真。最后，
                计算了不同迭代轮次的误差率，评估了算法的性能。
                ''')#换行\\\n或\n\n
    st.markdown(
                '''
                :blue[对于三维无源定位：]在该视角下，研究了多无人机的纯方位无源定位问题。首先，通过证明四
                点共面定理将三维球形编队降维为二维的圆形编队进行研究。其次，针对三架发射信
                号无人机(编号已知且位置无偏差)的情况，根据圆与三角形的几何关系建立四个定位模
                型并证明得到无人机无源定位定理。最后，针对两架发射信号无人机(编号已知且位置
                无偏差)的情况，根据几何关系证明得到无人机定位的最少数量定理。
                ''')
    st.markdown(
                '''
                :blue[对于求解准确位置：]在三维空间中每个质点都具有三个自由度，由三个基向量可以唯一确定个质点的坐标，即4架准确位置无人机。
                根据初始四个准确无人机空间坐标，来确定其他无人机复位后的空间坐标，由平动章动进动的合矩阵表达式知，这是一个求解线性方程组的问题。
                求解该方程组时采用迭代法，分别采用简单格式(Jacobi)、高斯-赛德尔(Guass-Seider)格式、松弛格式
                用计算精度(越高越好)和迭代次数(越少越好)做评估指标。寻找最佳松弛因子。这便是本研究设计的迭代算法。
                ''')
    st.markdown(
                '''
                :blue[对于纠正错误位置：]对于无人机最速定位问题。拆解为两个子问题：\n\n
                    \ta.是转换为三维空间内一曲面的最低点（改点定位的准确位置）采用共轭梯度迭代法。相较传统的最速梯度下降法，显著的避免了复位路径锯齿化的问题。\n\n
                    b.计算当前复位路径与“无人机-最佳位置连线”的所围面积。这成为“复位扫过面积”。其评估了无人机复位算法的优越性。越优良的复位路径应越逼近该连线。\n\n
                计算该面积采用数值积分的方法，具体是采用“高斯积分法”。它是数值积分中代数精度最高的算法。(n个插值节点有2n+1的代数精度)。相较传统的复化梯形积分和Simpson积分有更高的精度。此有完备的理论证明。
                ''')

if LocPri == "STEP1——二维无源定位":  
    st.title("二维无源定位")
    st.header('定理一')
    st.markdown(
                '''
                位于圆心的发射信号无人机(FY00)与其余任意两架位于圆周上的发射信号无人机(编号已知且位置无偏差)可唯一确定其余任一无人机位置。
                ''')
    st.caption(
                '''
                证明1对上述定理，根据发射信号无人机的位置分布布局，可将其分为“待定位无
                人机与任意两架发射信号的无人机不共线且位于劣弧范围内”、“待定位无人机与任意两
                架发射信号的无人机不共线且位于优弧范围内”、“待定位无人机与圆周上的两架无人机
                共线”、“待定位无人机与圆周和圆心两架无人机共线"4种情形。为方便讨论，不妨设
                为圆心，m、n、u分别为编号从小到大的无人机位点，如m表示FY03代号的尾数(即
                3)，以此类推。
                ''')
    case1, case2, case3, case4 = st.tabs(['情形1','情形2','情形3','情形4'])
    with case1:
        Case1Col1,Case1col2 = st.columns([0.25,0.75])
        with Case1Col1:
            st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/PositionCase1.png")) + '" width="300"></div>', unsafe_allow_html=True)
        with Case1col2:
            st.markdown("对应方程组")
            st.latex(r'\begin{cases}\begin{aligned}&d_{on}^2=d_{nv}^2+r^2-2rd_{nv}*\cos\angle nvo\left(\angle nvo=\pi-\angle vno-\angle nov\right)\\&d_{on}^2=d_{mn}^2+r^2-2rd_{mn}*\cos\angle mno\\&\left\{\angle mno=[\pi-\angle mno-(v-m)*40°-\angle nov]\right\}\end{aligned}\end{cases}')
            st.markdown("方程组有解且解唯一")

    with case2:
        Case2Col1,Case2col2 = st.columns([0.25,0.75])
        with Case2Col1:
            st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/PositionCase2.png")) + '" width="300"></div>', unsafe_allow_html=True)
        with Case2col2:
            st.markdown("对应方程组")
            st.latex(r'\begin{cases}\left.\begin{aligned}&d_{nv}{}^2-2rd_{nv}*\cos(\pi-\angle vno-\angle nov)=\\&d_{mn}{}^2-2rd_{mn}*\cos[\pi-\angle mno-(v-m)*40°-\angle nov]\\&\frac{d_{nv}}{\sin\left(\angle nov\right)}=\frac{r}{\sin\left(\angle vno\right)}\\&d_{mv}{}^2=d_{nv}{}^2+d_{mn}{}^2-2d_{nv}d_{mn}*\cos\left(\angle vnm\right)\\&\theta_1=(v-m)*40°-\angle nov\end{aligned}\right.\end{cases}')
            st.markdown("方程组有解且解唯一")

    with case3:
        Case3Col1,Case3col2 = st.columns([0.25,0.75])
        with Case3Col1:
            st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/PositionCase3.png")) + '" width="300"></div>', unsafe_allow_html=True)
        with Case3col2:
            st.markdown("对应方程组")
            st.latex(r'\begin{cases}\begin{aligned}&\angle mon=\pi-\angle mno-\angle omn\\&\theta_3=\angle mon+(m-0)*40°\\&\frac{r}{\angle mno}=\frac{d_{on}}{\angle omn}\end{aligned}\end{cases}')
            st.markdown("方程组有解且解唯一")

    with case4:
        Case4Col1,Case4col2 = st.columns([0.25,0.75])
        with Case4Col1:        
            st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/PositionCase4.png")) + '" width="300"></div>', unsafe_allow_html=True)
        with Case4col2:
            st.markdown("对应方程组")
            st.latex(r'\left.\left\{\begin{array}{c}\angle nov=\pi-\angle mov\\\angle ovn=\pi-\angle onv-\angle nov\\\frac{r}{\angle onv}=\frac{d}{\angle ovn}\\\theta_4=(v-0)*40°+\angle nov\end{array}\right.\right.')
            st.markdown("方程组有解且解唯一")

    st.divider()
    st.header('定理二')#定理2
    st.markdown(
                '''
                位于极轴上的发射信号无人机(FY00和FY01)与至少1架位于圆周上的发
                射信号无人机(位置无偏差但编号未知)唯一有效定位其余任一位置略有偏差且编号未
                知的无人机。
                ''')
    st.caption(
                '''
                Partl:编号未知发射信号无人机的数量与其编号位置。现假设可能有 2～5架编号
                未知发射信号的无人机，下面逐一进行讨论和验证。\n\n
                \t①2架编号未知发射信号的无人机，即除编号为FY00和FY01的无人机发射信号
                    外，不再需要额外发射信号的无人机。显然，当需定位无人机与编号为
                    FY00和FY01的无人机共线时只能确定其极角而不能确定其极径;当需定位无
                    人机与编号为FY00和FY01的无人机不共线时其极角与极径均不能确定。因此
                    此情形不能有效定位，故排除。\n\n
                    ②3架编号未知发射信号的无人机，即除编号为FY00和FY01的无人机发射信号
                    外，还需额外1架发射信号的无人机。为证明此情形的可行性，下面分两步进
                    行。\n\n
                ''')
    with st.expander('STEP1'):#定理2中展示的俩张图片Fir
        st.caption('规定正方向。规定逆时针方向为长度单位和角度的正方向，如图红箭头所示。')
        st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/Principle2Step1.png")) + '" width="800"></div>', unsafe_allow_html=True)
    with st.expander('STEP2'):#定理2中展示的俩张图片Sec
        st.caption('Step2:遍历搜寻所有无人机理想点位，可求得极角θ与(2π一θ)角构成的与发射信号无人机一一对应的的唯一组合，最后得到无人机的编号，如下公式所示。')
        st.latex(r'\begin{cases}FY01:\:0^{\circ}and\;360^{\circ}\\FY01:\:40^{\circ}and\;280^{\circ}\\\;\;\;\;\;\;\;\;\;\;\;\;\;\;\vdots\\FY09:\:280^{\circ}and\;40^{\circ}\end{cases}')                
    st.caption(
                '''                
                Part2:由 Partl 证得3可架编号未知发射信号的无人机可进行有效定位后，可套用
                问题一第一小问的模型，对三架之外的无人机进行有效定位。\n\n
                    ③4架编号未知发射信号的无人机，即除编号为FY00和FY01的无人机发射信号外，还需额外2架发射信号的无人机。因②中已证明4架无人机可实现有效定位，
                    因此此形不再考虑。\n\n                
                    ④5架编号未知发射信号的无人机情形同③，不再赘述。\n\n
                ''')
        

    st.divider()
    st.header('定理三')#定理三
    st.markdown(
                '''
                位于极轴上的发射信号无人机(FY00和FY01)与至少2架位于圆周上的发
                射信号无人机(位置有偏差且编号已知)唯一有效定位其余任一位置略有偏差且编号未知的无人机。
                ''')
    st.caption(
                '''
                由于待定位无人机分布位置的不同，其计算方式也有一定的差别，
                因此定理3的证明过程可分为以下两种情形分别进行讨论和分析。
                ''')
    Principle3Case1,Principle3Case2 = st.tabs(["情形一","情形二"])
    with Principle3Case1:
        Prin3Case1Col1,Prin3Case1col2 = st.columns([0.25,0.75])
        with Prin3Case1Col1:        
            st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/Principle3Case1.png")) + '" width="300"></div>', unsafe_allow_html=True)
        with Prin3Case1col2:
            st.markdown("对应方程组")
            st.latex(r'\begin{cases}\begin{aligned}&\cos\angle mno=\frac{d_{mn}{}^2+d_{mo}{}^2-r^2}{2d_{mn}d_{no}}\\&\cos\angle sno=\frac{d_{sn}{}^2+d_{no}{}^2-d_{so}{}^2}{2d_{sn}d_{no}}\\&\left(\frac{d_{no}{}^2+d_{so}{}^2-d_{nv}{}^2}{2d_{no}d_{so}}\right)^2+\left(\frac{d_{ns}{}^2*\sin\angle sno}{d_{so}}\right)=1\\&\frac{r}{\sin\angle mno}=\frac{d_{mn}}{\sin\angle mos*\left(\frac{d_{mn}{}^2+d_{so}{}^2-d_{ns}{}^2}{2d_{mo}d_{so}}\right)-\cos\angle mos*\left(\frac{d_{ns}*\sin\angle sno}{d_{so}}\right)}\end{aligned}\end{cases}')
            st.markdown("方程组有解且解唯一")

    with Principle3Case2:
        Prin3Case2Col1,Prin3Case2col2 = st.columns([0.25,0.75])
        with Prin3Case2Col1:        
            st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/Principle3Case2.png")) + '" width="300"></div>', unsafe_allow_html=True)
        with Prin3Case2col2:
            st.markdown("对应方程组")
            st.latex(r'\begin{cases}\begin{aligned}&\cos\angle mno=\frac{d_{mn}{}^2+d_{mo}{}^2-r^2}{2d_{mn}d_{no}}\\&\cos\angle sno=\frac{d_{sn}{}^2+d_{no}{}^2-d_{so}{}^2}{2d_{sn}d_{no}}\\&\left(\frac{d_{no}{}^2+d_{so}{}^2-d_{nv}{}^2}{2d_{no}d_{so}}\right)^2+\left(\frac{d_{ns}{}^2*\sin\angle sno}{d_{so}}\right)=1\\&\frac{r}{\sin\angle mno}=\frac{d_{mn}}{\sin\angle mos*\left(\frac{d_{mn}{}^2+d_{so}{}^2-d_{ns}{}^2}{2d_{mo}d_{so}}\right)-\cos\angle mos*\left(\frac{d_{ns}*\sin\angle sno}{d_{so}}\right)}\end{aligned}\end{cases}')
            st.markdown("方程组有解且解唯一,至此定理三得证")
if LocPri == "STEP2——三维无源定位":
    st.title("三维无源定位")

    st.write('本章研究的主题是三维空间下无人机编队的定位问题，为简化模型，现考虑一种特殊的三维无人机群编队队形－—球形编队队形，即一架无人机位于球心若干架无人机均匀位于球面的编队队形。让无人机均匀分布在球面上，最简单的方式是使用球面坐标系上的经纬度进行采样，但是这种方式得到的采样点在两极聚集但在赤道分散，分布并不均匀。本章中使用菲波那契网格(Fibonacci lattice）生成近似均匀的采样点(如图所示)，研究表明使用菲波那契网格测量球面上不规则图形的面积，与用经纬网格相比，误差可以减小40%"。因此可以认为其生成的点可以满足实际要求，即可以认为是均匀分布的，生成公式如下:')

    st.latex(r'\left.\left\{\begin{array}{c}x_n=\sqrt{1-{z_n}^2}\cdot\cos\left(2\pi n\phi\right)\\y_n=\sqrt{1-{z_n}^2}\cdot\sin\left(2\pi n\phi\right)\\z_n=\frac{2n-1}{N-1}\end{array}\right.\right.')

    st.write('$$其中，x_{n}，y_{n}，z_{n}表示第n个无人机的欧式空间x，y，z轴坐标，Φ是黄金分割比,约为0.618，N是排成球面所需要的总的粒子数,本章中的球形编队球面由N=999架直径可近似等于ε（无限小）的无人机组成。如图 27为球面分布999个点的效果，其任意两个节点到球心连线所形成的夹角相等，可看成三维球形无人机初始状态编队$$')

    st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/Demention3.png")) + '" width="800"></div>', unsafe_allow_html=True)

    st.markdown('''<p class = 'kai-font_center'>n球面上分布1000个点的效果图</p>''', unsafe_allow_html = True )

    st.divider()

    Demention3Col1,Demention3Col2,Demention3Col3,Demention3Col4 = st.tabs(['情形一','情形二','情形三','情形四'])
    with Demention3Col1 :
        st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/Demention3Case1.png")) + '" width="400"></div>', unsafe_allow_html=True)
        st.caption('可归结为如下的方程')
        st.latex(r'\left.\left\{\begin{array}{l}d_{nv}{}^2-2rd_{nv}\cos\left(\pi-\beta_2-\beta_4+\angle mon\right)\\=d_{nm}{}^2-2rd_{nm}\cos\left(\pi-\beta_1-\angle mon\right)\\\frac{d_{nv}}{\sin\angle nov}=\frac{r}{\sin\beta_2}(\angle nov=\beta_4-\angle mon)\\\frac{d_{nm}}{\sin\angle mon}=\frac{r}{\sin\beta_1}\end{array}\right.\right.')        
    with Demention3Col2 :
        st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/Demention3Case2.png")) + '" width="400"></div>', unsafe_allow_html=True)
        st.caption('可归结为如下的方程')
        st.latex(r'\left.\left\{\begin{array}{l}d_{nv}{}^2-2rd_{nv}\cos\left(\beta_4+\angle mon-\pi-\beta_2\right)\\=d_{nm}{}^2-2rd_{nm}\cos\left(\pi-\beta_1-\angle mon\right)\\\frac{d_{nv}}{\sin\angle nov}=\frac{r}{\sin\beta_2}\left(\angle nov=2\pi-\beta_4-\angle mon\right)\\\frac{d_{nm}}{\sin\angle mon}=\frac{r}{\sin\beta_1}\end{array}\right.\right.')
    with Demention3Col3 :
        st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/Demention3Case3.png")) + '" width="400"></div>', unsafe_allow_html=True)
        st.caption('可归结为如下的方程')
        st.latex(r'\Large\frac{r}{\sin\angle mno}=\frac{d_{on}}{\sin\angle omn}')
    with Demention3Col4 :
        st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/Demention3Case4.png")) + '" width="400"></div>', unsafe_allow_html=True)
        st.caption('可归结为如下的方程')
        st.latex(r'\Large\frac{r}{\sin\angle onv}=\frac{d_{on}}{\sin\angle ovn}')        
    st.divider()

    st.markdown(r':blue[定理一]过三维空间某一点A的三条直线分别为$$l_{1}$$,$$l_{2}$$和$$l_{3}$$,$$l_{1}$$和$$l_{2}$$的夹角为$$\alpha$$,$$l_{2}$$和$$l_{3}$$的夹角为$$\beta$$，$$l_{1}$$和$$l_{3}$$的夹角为$$\gamma$$($$\gamma$$>Q,$$\gamma$$>β)，$$l_{1}$$,$$l_{2}$$和$$l_{3}$$直线上分别存在B，C，D点,若满足$$\alpha$$+$$\beta$$=$$\gamma$$，则A，B，C，D四点共面。')

    st.markdown(r':blue[定理二]位于球心的发射信号无人机FY00与其余任意两架位于球面上的发射信号无人机(编号已知且位置无偏差)可唯一确定其余任一无人机的位置$$')

    st.markdown(r':blue[定理三]位于球心上的发射信号无人机FY00与位于球面上的发射信号无人机FY01与至少1架位于球面上的发射信号无人机(位置无偏差但编号未知)唯一有效定位其余任一位置略有偏差且编号未知的无人机。')


if LocPri == "STEP3——求解准确位置":
    st.title("求解准确位置")
    st.markdown(':blue[定理一：]无人机定位等效于刚体运动的复位')
    st.markdown('刚体是指任意两点间的距离不因力的作用而发生变化的质点组。而自由度是指确定一个系统运动状态所必须的，能够独立变化的物理量个数。对于三维空间中的球体，尤其是研究三维空间中的无人机相对定位问题时，可以把已复位的无人机形成的坐标框架视为刚体。在错位的无人机复位时，位置正确的无人机之间的相对距离不发生改变。它们间的整体运行可以视为刚体运动。\n\n任取刚体上三个不共线的点，便可以确定刚体运动的自由度。对于三个自由的质点应该存在 9 个自由度，如果用直角坐标系这九个自由度分别为$$x_1,y_1,z_1,x_2,y_2,z_2,x_3,y_3,z_3$$对于刚体来说，这三个质点两两之间的距离保持不变，故有三个约束方程')
    st.latex(r'(x_1-x_2)^2+(y_1-y_2)^2+(z_1-z_2)^2=r_{12}^2\\(x_2-x_3)^2+(y_2-y_3)^2+(z_2-z_3)^2=r_{23}^2\\(x_1-x_3)^2+(y_1-y_3)^2+(z_1-z_3)^2=r_{13}^2')
    st.markdown(':blue[定理二：]三维空间中无人机定位可视为刚体的定点转动，其一定可以分解为三种独立的运动，分别为 自转(图 a)、进动(图 b)、章动(图 c)')
    st.markdown('<div class="center"><img src="data:image/png;base64,' + image_to_base64(Image.open("./images/RigidMotionClassification.png")) + '" width="1000"></div>', unsafe_allow_html=True)

    RigidMotionTranslation = st.expander(label=r'$$\begin{aligned}&\text{对于图(a)：首先将坐标轴绕着 z 方向旋转一个角度 }\phi(0\thicksim 2\pi)\text{,这称之为进动,}  \phi\text{ 叫做进动角.}\text{此时的刚体坐标系为 }O\xi\eta\zeta.\text{三个方}\\&\text{向上的单位矢量为 }\vec{e}_{1}^{\prime},\vec{e}_{2}^{\prime},\vec{e}_{3}^{\prime}\text{.进动对应的矩阵形式如下:}\end{aligned}$$',expanded = True)
    RigidMotionPrecession = st.expander(label=r'$$\begin{aligned}&\text{对于图(b)：将坐标系 }O\xi\eta\zeta\text{绕着}\xi\text{轴旋转一个角度 }\theta(0\thicksim\pi)\text{,称为章动,}\theta\text{为章动角}_{.}\text{转动之后的刚体坐标系为}O\xi^{\prime}\eta^{\prime}\zeta^{\prime}.\text{三个方向上的}\\&\text{单位矢量为}\vec{e}_{1}^{\prime\prime},\vec{e}_{2}^{\prime\prime},\vec{e}_{3}^{\prime\prime}.\text{图中的 ON 称为节线,为章动时的转轴.章动对应的矩阵形式如下:}\end{aligned}$$',expanded = True)
    RigidMotionNutation = st.expander(label=r'$$\begin{aligned}&\text{对于图(c)：将坐标系}O\xi^{\prime}\eta^{\prime}\zeta^{\prime}\text{再绕着}\zeta^{\prime}\text{轴旋转一个角度}\psi(0\thicksim2\pi)\text{ ,称为自转,}\psi\text{为自转角}_{.}\text{转动后的刚体坐标系为}Ox^{\prime}y^{\prime}z^{\prime}.\text{三个方向}\\&\text{上的单位矢量为}\vec{e}_{1}^{\prime\prime\prime},\vec{e}_{2}^{\prime\prime\prime},\vec{e}_{3}^{\prime\prime\prime}.\text{即最终状态的刚体系.自转对应的矩阵形式如下}\end{aligned}$$',expanded = True)

    with RigidMotionTranslation:
        st.latex(r'\begin{aligned}&\text{由进动过程有}\\&&\begin{pmatrix}\vec{e}_1^{\prime}\\\vec{e}_2^{\prime}\\\vec{e}_3^{\prime}\end{pmatrix}=\begin{pmatrix}cos\phi&sin\phi&0\\-sin\phi&cos\phi&0\\0&0&1\end{pmatrix}\begin{pmatrix}\vec{e}_1\\\vec{e}_2\\\vec{e}_3\end{pmatrix}=T_1\begin{pmatrix}\vec{e}_1\\\vec{e}_2\\\vec{e}_3\end{pmatrix}\end{aligned}')
    with RigidMotionPrecession:
        st.latex(r'\begin{aligned}&\text{由章动过程有}\\&&\begin{pmatrix}\vec{e}_1^{\prime\prime}\\\vec{e}_2{}^{\prime\prime}\\\vec{e}_3{}^{\prime\prime}\end{pmatrix}=\begin{pmatrix}1&0&0\\0&cos\theta&sin\theta\\0&-sin\theta&cos\theta\end{pmatrix}\begin{pmatrix}\vec{e}_1{}^{\prime}\\\vec{e}_2{}^{\prime}\\\vec{e}_3{}^{\prime}\end{pmatrix}=T_2\begin{pmatrix}\vec{e}_1{}^{\prime}\\\vec{e}_2{}^{\prime}\\\vec{e}_3{}^{\prime}\end{pmatrix}\end{aligned}')
    with RigidMotionNutation:
        st.latex(r'\begin{aligned}&\text{由自转过程有}\\&&\begin{pmatrix}\vec{e}_1^{\prime\prime\prime}\\\vec{e}_2^{\prime\prime\prime}\\\vec{e}_3^{\prime\prime\prime}\end{pmatrix}=\begin{pmatrix}cos\psi&sin\psi&0\\-sin\psi&cos\psi&0\\0&0&1\end{pmatrix}\begin{pmatrix}\vec{e}_1^{\prime\prime}\\\vec{e}_2^{\prime\prime}\\\vec{e}_3^{\prime\prime}\end{pmatrix}=T_3\begin{pmatrix}\vec{e}_1^{\prime\prime}\\\vec{e}_2^{\prime\prime}\\\vec{e}_3^{\prime\prime}\end{pmatrix}\end{aligned}')
    st.markdown('作变量代换,令:')
    st.markdown(r"$$\begin{gathered}\\\begin{cases}&a_{11}=2(x_3-x_1)\\&a_{12}=2(y_3-y_1)\\&a_{13}=2(z_3-z_1)\\&b_1=r_1^2-r_3^2-x_1^2+x_3^2-y_1^2+y_3^2-z_1^2+z_3^2\\&a_{21}=2(x_3-x_2)\\&a_{22}=2(y_3-y_2)\\&a_{23}=2(z_3-z_2)\\&b_{2}=r_2^2-r_3^2-x_2^2+x_3^2-y_2^2+y_3^2-z_2^2+z_3^2\end{cases}\\\\\text{可得如下线性方程组：}\\\\\begin{cases}p_1r-q_1=(pcos\psi cos\phi-sin\psi sin\phi cos\theta)x_1+(sin\psi cos\theta+cos\psi sin\phi cos\theta)y_1+cos\phi sin\theta\\p_2r-q_2=(-cos\psi sin\phi-sin\psi cos\varphi cos\theta)x_2+(-sin\psi sin\phi+cos\psi cos\theta)y_2+cos\phi sin\theta z_2\\p_3r-q_3=sin\psi sin\theta x_3-cos\psi sin\theta y_3+cos\theta z_3&\end{cases}\\\begin{bmatrix}p_1\\p_2\\p_3\end{bmatrix}r-\begin{bmatrix}q_1\\q_2\\q_3\end{bmatrix}=T\cdot\begin{bmatrix}x\\y\\z\end{bmatrix}\end{gathered}$$")
    st.markdown('这是一个线性方程组.即在球面均匀分布的情况下，求解每架无人机准确位置的问题可以归结为一个求解线性方程组的问题.')
    st.divider()
    st.header('STEP1迭代格式的选取')
    st.subheader(':gray[高斯赛德尔迭代(Guass-Seidel)]')
    st.code(body = '''
            import numpy as np

            def gauss_seidel(A, b, tolerance=1e-10, max_iterations=1000):
                n = len(b)
                x = np.zeros_like(b, dtype=np.double)
                
                for k in range(max_iterations):
                    x_new = np.copy(x)
                    
                    for i in range(n):
                        s1 = np.dot(A[i, :i], x_new[:i])
                        s2 = np.dot(A[i, i + 1:], x[i + 1:])
                        x_new[i] = (b[i] - s1 - s2) / A[i, i]
                    
                    if np.linalg.norm(x_new - x, ord=np.inf) < tolerance:
                        return x_new
                    
                    x = x_new
                
                raise Exception("Gauss-Seidel method did not converge")

            solution = gauss_seidel(T, X)
            print("Solution:", solution)
            ''',language='python')
    st.subheader(':gray[松弛迭代(SOR)]')
    st.code(body = '''
            import numpy as np
            import math
            import sys

            def DLU(A):
                D=np.zeros(np.shape(A))
                L=np.zeros(np.shape(A))
                U=np.zeros(np.shape(A))
                for i in range(A.shape[0]):
                    D[i,i]=A[i,i]
                    for j in range(i):
                        L[i,j]=-A[i,j]
                    for k in list(range(i+1,A.shape[1])):
                        U[i,k]=-A[i,k]
                L=np.mat(L)
                D=np.mat(D)
                U=np.mat(U)
                return D,L,U
            
            def SOR(A,b,x0,w,maxN,p):  #x0为初始值，maxN为最大迭代次数，p为允许误差
                D,L,U=DLU(A)
                if len(A)==len(b):
                    M=np.linalg.inv(D-w*L)
                    M=np.mat(M)
                    B=M * ((1-w)*D + w*U)
                    B=np.mat(B)
                    f=w*M*b
                    f=np.mat(f)
                else:
                    print('维数不一致')
                    sys.exit(0)  # 强制退出
                
                a,b=np.linalg.eig(B) #a为特征值集合，b为特征值向量
                c=np.max(np.abs(a)) #返回谱半径
                if c<1:
                    print('迭代收敛')
                else:
                    print('迭代不收敛')
                    sys.exit(0)  # 强制退出
            #以上都是判断迭代能否进行的过程，下面正式迭代
                k=0
                while k<maxN:
                    x=B*x0+f
                    k=k+1
                    eps=np.linalg.norm(x-x0,ord=2)
                    if eps<p:
                        break
                    else:
                        x0=x
                return k,x
            
            A = np.mat([[-4,1,1,1],[1,-4,1,1],[1,1,-4,1],[1,1,1,-4]])
            b = np.mat([[1],[1],[1],[1]])
            x0=np.mat([[0],[0],[0],[0]])
            maxN=100
            p=0.00001
            w=1.3
            print("原系数矩阵a:")
            print(A)
            print("原值矩阵b:")
            print(b)
            k,x=SOR(A,b,x0,w,maxN,p)
            print("迭代次数")
            print(k)
            print("数值解")
            print(x)''',language="python")
    st.header('STEP2算法终止条件')
    st.markdown('算法终止的条件是借助了:red[柯西收敛准则]。伴随迭代的进行时，若两两相邻的一列解的波动范围在给定区间内，此时认为解已经:red[近似收敛]。得出的该解认为是该线性方程组的解。即:')
    st.latex(r'|x_{m}-x_{n}|\le\epsilon')
    st.markdown('这里的$$\epsilon$$是预先给定的波动范围也称允许误差。')
    st.header('STEP3求解结果检验')
    st.markdown('计算出的定位位置是否恰好能均匀分布在上球面呢，这需要对结果进行检验。这里利用微分几何的知识。计算完成后，将无人机布置所得的解位置上，则它们构成一个折面，当无人机的数量越多，折面越光滑，可用近似曲面代替。对曲面上的无人机位置进行随机采样。进而计算曲面上的:red[第一基本型]和:red[第二基本型]。\n\n :blue[$$定理:$$]$$第一基本型和第二基本型的比值是与u,v无关的定函数f时，曲面 r(u,v)为球面$$')
    
if LocPri == "STEP4——纠正错误位置":
    st.title("纠正错误位置")
    st.header('STEP1使用共轭梯度下降法生成复位路径')
    st.subheader(':gray[算法步骤:]')

    st.markdown('$$STEP1.设定初值$$')
    st.latex(r'\vec{d}_0=\vec{r}_0=\vec{b}-A\vec{x}_0')

    st.markdown(r'$$STEP2.计算系数\alpha$$')
    st.latex(r'\alpha_{i}=\frac{\vec{r}_{i+1}^{T}\vec{r}_{i+1}}{\vec{d}_{i}^{T}A\vec{d}_{i}}')

    st.markdown('$$STEP3.朝梯度下降方向迭代一步$$')
    st.latex(r'\vec{x}_{i+1}=\vec{x}_{i}-\alpha_{i}\vec{d}_{i}')

    st.markdown('$$STEP4.计算残差$$')
    st.markdown('')
    st.markdown(r'$$\\\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;a.如果迭代次数达到终止条件,执行:$$')
    st.latex(r'\vec{r}_{i+1}=\vec{r}_{i}-\alpha_{i}A\vec{x}')
    st.markdown(r'$$\\\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;b.否则执行:$$')
    st.latex(r'\vec{r}_{i+1}=\vec{r}_{i}-\alpha_{i}Ad')

    st.markdown(r'$$STEP5.计算系数\beta$$')
    st.latex(r'\beta_{i+1}=\frac{\vec{r}_{i+1}^T\vec{r}_{i+1}}{\vec{r}_i^T\vec{r}_i}')

    st.markdown(r'$$STEP6.计算搜索方向\vec{d}$$')
    st.latex(r'\vec{d}_{i+1}=\vec{r}_{i+1}+\beta_{i+1}\vec{d}_{i}')

    st.markdown('$$STEP7.反复执行2-6步直至$$')
    st.latex(r'\vec{r}_i\leq\epsilon')

    st.header('STEP2使用高斯积分法计算复位扫过面积')
    st.markdown('')
    st.subheader(':gray[算法步骤:]')
    st.markdown(
            '''对生成的结果需进行检验，计算器复位面积。在对其余路径方案的扫过面积
            进行对比，确保生成结果对应得扫过面积面积最小。
            采用高斯积分法进行计算。\n\n
            基本思想：将被积函数在某些节点上的函数值加权求和并以该和值作为积分的近似值''')
    st.latex(r'''\int_a^bf(x)dx\approx\sum_{k=0}^nA_kf(x_k),a\leq x_0<x_0<\cdots<x_n\leq b''')
    st.markdown('''数值求积公式是近似的，该如何刻画求积公式的精确程度呢？为此引入“代数精度”的概念，借助多项式来刻画求积公式的精确程度。\n\n
            定义：如果求积公式对所有次数不超过m的多项式是精确的，但对m+1次多项式不精确，则称该求积公式具有m次代数精度。''')
    st.divider()
    st.markdown('$$STEP1.计算权函数$$')
    st.markdown(r'$$\begin{aligned}&\text{定义:区间}[a,b]\text{上的非负函数}\rho(x)\text{ ,若它满足}:\\\\&\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;(1)\int_a^b\lvert x\rvert^n\rho(x)dx\text{对一切非负整数}n\text{可积且有限};\\&\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;(2)\text{假设对某个非负的连续函数}g(x),\int_a^b\rho(x)g(x)dx=0\text{ ,则在区间}[a,b]\text{上}\\\\&\text{函数}g(x)=0\text{ ,则称}\rho(x)\text{为权函数}\end{aligned}$$')
    st.divider()    
    st.markdown('$$STEP2.计算正交多项式$$')
    st.markdown(r'$$\begin{aligned}&\text{定义:在}[a,b]\text{上的函数系}\{g_l(x)\}_{l=0}^n,\text{如果满足:}\\\\&\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;(1)g_l(x)=\sum_{k=0}^l\alpha_kx^k\text{恰为}l\text{次多项式,即}\alpha_l\neq0;\\\\&\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;\;(2)(g_i,g_j)=\begin{cases}0,&i\neq j\\\int_a^b&\rho(x)g_i^2&(x)dx>0,&i=j&\end{cases}\\\\&\text{则函数系}\{g_{l}(x)\}_{l=0}^{n}\text{称为}P_{n}\text{的带权}\rho(x)\text{ 正交基 }(g_{l}(x)\text{称为}[a,b]\text{上的带权}l\text{次正交多项式。}\end{aligned}$$')
 








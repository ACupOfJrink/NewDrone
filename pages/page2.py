import streamlit as st
import numpy as np
import pandas as pd
import time
import pydeck as pdk
from urllib.error import URLError
import altair as alt
from typing import Any
import vega_datasets 
import plotly.express as px
import plotly.figure_factory as ff

# st.header("这是页面 2")

# if st.button("GoTo Page 1"):

#     st.switch_page("pages/page1.py")
#st.set_page_config(page_title='算法对比与数据验证', layout="wide")

st.title('云端模拟与数据验证')

#use markdown 创建链接
st.markdown ('''_模拟仿真数据来源:_ [访问](https://www.scidb.cn/en/detail?dataSetId=720626420933459968&version=V1&dataSetType=journal&tag=2&language=zh_CN)''')
st.caption ('''数据集内容：红外小目标检测跟踪,是远程精确打击、空天攻防对抗,和遥感情报侦察等军事应用中的重要研究内容。
针对当前红外实测数据,样本匮乏的情况,收集了一架或多架,固定机翼无人机目标为检测对象的算法测试数据集。
数据集数量：数据集获取场景：涵盖了天空、地面等多种场景,共计 22 段数据、30 条航迹、16177 帧图像、16944 个目标。
数据集功能以及目标检测''')

st.markdown ('''_方向定位数据来源:_ [访问](https://hf-mirror.com/datasets/Timdb/AE4317-cyberzoo-tudelft/tree/main/data)''')
st.caption ('''该数据集包含在Delft University of Technology的Cyberzoo中拍摄的图像,包括82%的真实图像和18%的模拟器图像。数据集用于训练微型无人机在Cyberzoo环境中的障碍物避免,通过单眼深度图生成的标签进行标注。数据集分为训练集（90%）和测试集（10%）,每张图像的元数据中嵌入了标签,指示无人机应旋转或飞行的方向。''')

# 侧边栏
with st.sidebar:
    LocPri = st.sidebar.selectbox(
        "选择训练集数据",
        (
            "A:zip1——2024.10.01",
            "B:zip2——2024.03.17",
            "C:zip3——2023.11.09",
            "D:zip4——2023.10.09",
            "E:zip5——2023.06.23"
        ),
    )

    # 分隔线 
    st.divider()

    # 是否使用模拟数据库
    DirPri = st.sidebar.radio(
        "是否使用模拟数据库",
        ("是", "否"),
    )

    # 是否预先优化参数
    SelfPro = st.sidebar.radio(
        "是否预先优化参数",
        ("采用", "不采用"),
    )

    #环境调参
    st.write('环境参数设置')

    windspeed = st.slider('风速 (m/s)',max_value=10.0,min_value=0.0,value = 5.4)
    Altitude = st.slider('海拔 (ft)',max_value=5000,min_value=-150,value = 854)
    Temperature = st.slider('温度(C)',max_value=40.0,min_value=-20.0,value = 25.1)
    Cichang = st.slider('磁场(Gauss)',max_value=0.65,min_value=0.15,value = 0.3)
    Obstacle = st.select_slider('障碍物等级',[0,1,2,3,4,5],value = 3)

    st.divider()  
    

# 绘制地图
st.divider()    
st.header(f"数据集： {LocPri}")

st.markdown("_数据采样地点:世界地图展示_")
map_data = pd
map_data = pd.DataFrame(
    np.random.randn(1000, 2) / [50,50] + [37.76,-122.4],
    columns=['lat', 'lon']
)
#注意：map的数据集中必须要有lat和lon固定列名的两列数据作为经纬度
st.map(map_data)


#动画演示
st.divider()  
st.header("定位算法模拟演示")
st.markdown("_数据加载与模拟运行需要一定时间,请耐心等待_")
#横向的标签页
tab1, tab2, tab3, tab4 = st.tabs(["编队分形模拟运行", "编队任务响应时间", "编队跨城市路线模拟", "算法自适应度评分对比"])
 
with tab1:
    tab1button = st.button('点击开始运行',key = 'tab1')
    if tab1button :
        #st.write("可以调用方法一")
        # Interactive Streamlit elements, like these sliders, return their value.
        # This gives you an extremely simple interaction model.
        
        st.sidebar.write("_:blue[编队分形模拟运行]_")    
        iterations = st.sidebar.slider("细节层次(LOD)", 2, 20, 10, 1)
        separation = st.sidebar.slider("分形分离参数(Separation)", 0.7, 2.0, 0.7885)
        st.caption("此动画展示了在纯无源情况下初始无人机在基于Julia集模拟运行的定位过程,可以通过左侧“细节层次(LOD)”与“分离参数”调控算法的运行过程")
        # Non-interactive elements return a placeholder to their location
        # in the app. Here we're storing progress_bar to update it later.
        progress_bar = st.sidebar.progress(0)

        # These two elements will be filled in later, so we create a placeholder
        # for them using st.empty()
        frame_text = st.sidebar.empty()
        image = st.empty()

        m, n, s = 960, 640, 400
        x = np.linspace(-m / s, m / s, num=m).reshape((1, m))
        y = np.linspace(-n / s, n / s, num=n).reshape((n, 1))

        for frame_num, a in enumerate(np.linspace(0.0, 4 * np.pi, 100)):
            # Here were setting value for these two elements.
            progress_bar.progress(frame_num)
            frame_text.text("任务加载进度 %i/100" % (frame_num + 1))

            # Performing some fractal wizardry.
            c = separation * np.exp(1j * a)
            Z = np.tile(x, (n, 1)) + 1j * np.tile(y, (1, m))
            C = np.full((n, m), c)
            M: Any = np.full((n, m), True, dtype=bool)
            N = np.zeros((n, m))

            for i in range(iterations):
                Z[M] = Z[M] * Z[M] + C[M]
                M[np.abs(Z) > 2] = False
                N[M] = i

            # Update the image placeholder by calling the image() function on it.
            image.image(1.0 - (N / N.max()), use_column_width=True)

        # We clear elements by calling empty on them.
        progress_bar.empty()
        frame_text.empty()

        # Streamlit widgets automatically run the script from top to bottom. Since
        # this button is not connected to any other logic, it just causes a plain
        # rerun.
        st.button("再次运行",key=1)
















    
with tab2:
    tab2button = st.button('点击开始运行',key='tab2')
    if tab2button :
        st.sidebar.write("_:blue[编队任务响应时间]_")
        st.caption("此动画展示了在纯无源情况下在数据库中随机挑选500个子任务下,定位算法的平均响应时间(对数刻度下).注意可以在左侧选择是否加载模拟数据库")
        with st.spinner('加载中...'):
            time.sleep(2)
        
        progress_bar = st.sidebar.progress(0)
        status_text = st.sidebar.empty()
        last_rows = np.random.randn(1, 1)
        chart = st.line_chart(last_rows)

        for i in range(1, 101):
            new_rows = last_rows[-1, :] + np.random.randn(5, 1).cumsum(axis=0)
            status_text.text("%i%% 模拟已完成" % i)
            chart.add_rows(new_rows)
            progress_bar.progress(i)
            last_rows = new_rows
            time.sleep(0.05)

        progress_bar.empty()

        # Streamlit widgets automatically run the script from top to bottom. Since
        # this button is not connected to any other logic, it just causes a plain
        # rerun.
        st.button("再次运行",key=2)





















with tab3: 
    tab3button = st.button('点击开始运行',key='tab3')
    if tab3button :    
        st.sidebar.write("_:blue[编队跨城市路线模拟]_")
        st.caption("此地图路线展示了在跨城市路线下,无人机编队的跨城市飞行的停泊、定位、定向与队流情况.您可以通过左侧的地图面板选择查看特定细节")
        @st.cache_data
        def from_data_file(filename):
            url = (
                "https://raw.githubusercontent.com/streamlit/"
                "example-data/master/hello/v1/%s" % filename
            )
            return pd.read_json(url)

        try:
            ALL_LAYERS = {
                "编队停泊点": pdk.Layer(
                    "HexagonLayer",
                    data=from_data_file("bike_rental_stats.json"),
                    get_position=["lon", "lat"],
                    radius=200,
                    elevation_scale=4,
                    elevation_range=[0, 1000],
                    extruded=True,
                ),
                "编队定位点": pdk.Layer(
                    "ScatterplotLayer",
                    data=from_data_file("bart_stop_stats.json"),
                    get_position=["lon", "lat"],
                    get_color=[200, 30, 0, 160],
                    get_radius="[exits]",
                    radius_scale=0.05,
                ),
                "编队定向点": pdk.Layer(
                    "TextLayer",
                    data=from_data_file("bart_stop_stats.json"),
                    get_position=["lon", "lat"],
                    get_text="name",
                    get_color=[0, 0, 0, 200],
                    get_size=10,
                    get_alignment_baseline="'bottom'",
                ),
                "编队飞行队流": pdk.Layer(
                    "ArcLayer",
                    data=from_data_file("bart_path_stats.json"),
                    get_source_position=["lon", "lat"],
                    get_target_position=["lon2", "lat2"],
                    get_source_color=[200, 30, 0, 160],
                    get_target_color=[200, 30, 0, 160],
                    auto_highlight=True,
                    width_scale=0.0001,
                    get_width="outbound",
                    width_min_pixels=3,
                    width_max_pixels=30,
                ),
            }
            st.sidebar.markdown("### 地图面板")
            selected_layers = [
                layer
                for layer_name, layer in ALL_LAYERS.items()
                if st.sidebar.checkbox(layer_name, True)
            ]
            if selected_layers:
                st.pydeck_chart(
                    pdk.Deck(
                        map_style=None,
                        initial_view_state={
                            "latitude": 37.76,
                            "longitude": -122.4,
                            "zoom": 11,
                            "pitch": 50,
                        },
                        layers=selected_layers,
                    )
                )
            else:
                st.error("Please choose at least one layer above.")
        except URLError as e:
            st.error(
                """
                **This demo requires internet access.**
                Connection error: %s
            """
                % e.reason
            )





with tab4:
    tab4button = st.button('点击开始运行',key='tab4')
    if tab4button :
        st.sidebar.write("_:blue[算法自适应度评分对比]_")
        st.caption("Cyberzoo与Scibd共提供全球67个国家的地形模拟数据,您可以通过在选项框中选择至少一个国家进而将其在数据库中的对应用于算法模拟.算法会生成自适应度评分,其值越高,越表明算法在不同地域、环境下的鲁棒性与稳健性越佳.通过不同国家数据的模拟,能对比算法在不同地域条件下的模拟性能")

        # @st.cache_data
        # def get_UN_data():
        #     AWS_BUCKET_URL = "https://streamlit-demo-data.s3-us-west-2.amazonaws.com"
        #     df = pd.read_csv(AWS_BUCKET_URL + "/agri.csv.gz")
        #     return df.set_index("Region")

        # try:
        #     df = get_UN_data()
        #     countries = st.multiselect(
        #         "选择数据库来源(国家来源)", list(df.index), ["China", "United States of America"]
        #     )
        #     if not countries:
        #         st.error("请至少选择一个国家.")
        #     else:
        #         data = df.loc[countries]
        #         data /= 1000000.0
        #         st.write("### 自适应度评分", data.sort_index())

        #         data = data.T.reset_index()
        #         data = pd.melt(data, id_vars=["index"]).rename(
        #             columns={"index": "year", "value": "Gross Agricultural Product ($B)"}
        #         )
        #         chart = (
        #             alt.Chart(data)
        #             .mark_area(opacity=0.3)
        #             .encode(
        #                 # x="迭代次数:T",
        #                 # y=alt.Y("自适应度评分:Q", stack=None),
        #                 # color="地区:N",
        #                 x="year:T",
        #                 y=alt.Y("Gross Agricultural Product ($B):Q", stack=None),
        #                 color="Region:N",
        #             )
        #         )
        #         st.altair_chart(chart, use_container_width=True)
        # except URLError as e:
        #     st.error(
        #         """
        #         **This demo requires internet access.**
        #         Connection error: %s
        #     """
        #         % e.reason
        #     )

        # 获取数据并缓存
        @st.cache_data
        def get_UN_data():
            AWS_BUCKET_URL = "https://streamlit-demo-data.s3-us-west-2.amazonaws.com"
            df = pd.read_csv(AWS_BUCKET_URL + "/agri.csv.gz")
            return df.set_index("Region")

        # 处理数据并绘制图表
        try:
            # 获取数据
            df = get_UN_data()

            # 选择国家
            countries = st.multiselect(
                "选择数据库来源(国家来源)", list(df.index), ["China", "United States of America"]
            )

            if not countries:
                st.error("请至少选择一个国家.")
            else:
                # 选择的国家数据
                data = df.loc[countries]
                
                # 修改年份数据
                data.columns = data.columns.astype(int)  # 确保列名为整数
                data.columns = (data.columns - 1960) * 100  # 年份 - 1960 后乘以 100

                data /= 1000000.0  # 自适应度评分除以1000000
                # 展示表格
                st.write("### 自适应度评分", data.sort_index())

                # 转置数据并调整格式
                data = data.T.reset_index()
                data = pd.melt(data, id_vars=["index"]).rename(
                    columns={"index": "迭代次数", "value": "自适应度评分"}
                )

                # 绘制折线图
                chart = (
                    alt.Chart(data)
                    .mark_area(opacity=0.3)
                    .encode(
                        x="迭代次数:T",  # x轴为迭代次数
                        y=alt.Y("自适应度评分:Q", stack=None),  # y轴为自适应度评分
                        color="Region:N",  # 图例中显示“国家”
                    )
                )
                
                # 显示图表
                st.altair_chart(chart, use_container_width=True)

        except URLError as e:
            st.error(
                f"""
                **This demo requires internet access.**
                Connection error: {e.reason}
                """
            )



# # 绘图的类型放在session中
# if "graph" not in st.session_state:
#     st.session_state.graph = ""
st.divider()  
st.header("算法对比") 
# # 第一列
# def column_1():
#     st.header("1. 选择数据")
#     st.selectbox(
#         "选择数据集?",
#         (
#             "手写数字数据",
#             "房屋成交数据",
#             "股市交易数据",
#         ),
#     )
#     # 随机模拟的数据
#     data = pd.DataFrame(np.random.randn(5, 3), columns=["A", "B", "C"])
#     st.table(data)
 
 
# def column_2():
#     st.header("2. 配置数据")
#     graph = st.radio(
#         "图形类型: ",
#         ("折线图", "柱状图", "散点图"),
#     )
 
#     st.session_state.graph = graph
 
 
# def column_3():
#     st.header("3. 绘制图形")
 
#     chart_data = pd.DataFrame(np.random.randn(20, 3), columns=["A", "B", "C"])
#     if st.session_state.graph == "散点图":
#         st.scatter_chart(chart_data)
 
#     if st.session_state.graph == "折线图":
#         st.line_chart(chart_data)
 
#     if st.session_state.graph == "柱状图":
#         st.bar_chart(chart_data) 
 
# col1, col2, col3 = st.columns(3)
 
# with col1:
#     column_1()
 
# with col2:
#     column_2()
 
# with col3:
#     column_3()

source = vega_datasets.data.cars()
source.rename(columns={'Miles_per_Gallon': '平均均方误差'}, inplace=True)
source.rename(columns={'Horsepower': '迭代次数'}, inplace=True)
source.rename(columns={'Origin': '算法类型'}, inplace=True)

# 2. 将特定列中值为'A'的项替换为'B'
source['算法类型'] = source['算法类型'].replace('USA', '无源定位算法')
source['算法类型'] = source['算法类型'].replace('Japan', '传统球形定位算法')
source['算法类型'] = source['算法类型'].replace('Europe', '传统拟合定位算法')
chart = {
    "mark": "point",
    "encoding": {
        "x": {
            "field": "迭代次数",
            "type": "quantitative",
        },
        "y": {
            "field": "平均均方误差",
            "type": "quantitative",
        },
        "color": {"field": "算法类型", "type": "nominal"},
        "shape": {"field": "算法类型", "type": "nominal"},
    },
}

tab1, tab2 = st.tabs(["Streamlit 主题（默认）", "Vega-Lite 原生主题"])

with tab1:
    # 使用 Streamlit 主题，这是默认主题，您也可以省略 theme 参数
    st.vega_lite_chart(
        source, chart, theme="streamlit", use_container_width=True
    )

with tab2:
    st.vega_lite_chart(
        source, chart, theme=None, use_container_width=True
    )

st.markdown('_响应时间分布 :分布越靠左,说明算法响应时间越及时,运算性能更佳_')
# 添加直方图数据
x1 = np.random.randn(200) - 2
x2 = np.random.randn(200)
x3 = np.random.randn(200) + 2

# 将数据分组在一起
hist_data = [x1, x2, x3]
group_labels = ['无源定位算法', '传统球形算法', '传统拟合算法']

# 使用自定义 bin_size 创建 distplot
fig = ff.create_distplot(hist_data, group_labels, bin_size=[.1, .25, .5])

# 绘制图表
st.plotly_chart(fig, use_container_width=True)

df = px.data.gapminder()

df.rename(columns={'lifeExp': '稳健性'}, inplace=True)
df.rename(columns={'gdpPercap': '迭代次数'}, inplace=True)
df.rename(columns={'continent': '算法类型'}, inplace=True)

df['算法类型'] = df['算法类型'].replace('Asia', '无源定位算法')
df['算法类型'] = df['算法类型'].replace('Americas', '传统球形算法')
df['算法类型'] = df['算法类型'].replace('Europe', '传统边界算法')
df['算法类型'] = df['算法类型'].replace('Africa', '传统拟合算法')
df['算法类型'] = df['算法类型'].replace('Oceania', '传统神经网络算法')
fig = px.scatter(
    df.query("year==2007"),
    x="迭代次数",
    y="稳健性",
    size="pop",
    color="算法类型",
    hover_name="country",
    log_x=True,
    size_max=60,
)

tab1, tab2 = st.tabs(["Streamlit 主题（默认）", "Plotly 原生主题"])
with tab1:
    # 使用 Streamlit 主题，这是默认的方式，也可以省略 theme 参数
    st.plotly_chart(fig, theme="streamlit", use_container_width=True)
with tab2:
    # 使用 Plotly 的原生主题
    st.plotly_chart(fig, theme=None, use_container_width=True)


# Ocr-Character-Detection
利用centernet定位和crnn识别网络,识别护照上的两行机读码,提取出关键信息,并在海思平台上优化和部署.此工程只给出了海思上部署代码,由于涉及到保密问题,部分权重文件为empty.
# centernet pytorch训练
根据[centernet官方工程](https://github.com/Duankaiwen/CenterNet)训练数据,再利用[模型转换工具](https://github.com/xxradon/PytorchToCaffe),将pytorch模型转成caffe模型.检测机读码上的四个角字符,共四个框.再通过后处理截图,旋转缩放到crnn网络所需的数据要求.
# crnn caffe训练
根据[caffe官方工程](https://github.com/senlinuc/caffe_ocr)训练ocr字符识别网络,分了39个类别(原文给的是5990个类别,后来根据项目缩减成39类).其中训练的时候需要将训练数据字符长度对齐,否则训练无法收敛.识别字符种类为39类：其中包括0-9个数字、26个英文字母加上’<’、’-’、blank字符标签。整个工程输入原图片像素大小需要保证为偶数（最好大于480*480），其中centernet定位网络输入图片尺寸为480*480，crnn识别网络输入图片尺寸为1216 * 32，单个字符在原图上大小约为32 * 32.
分析整个网络,宽度下采样8倍,最终维度变成140;高度先下采样8倍,再通过一个pooling层直接下采样4倍,最终维度变成1;通道即是特征数,最终维度变成128,本人尝试过改大这个数值,效果提升并不明显.最后再经过fc加CTC层.

# 耗时对比
测试结果可以看到：处理单张图片共耗时85ms左右，其中：ive前处理耗时8ms左右，NNIE中Centernet定位网络耗时47ms，NNIE中单个CRNN识别网络耗时12ms（CRNN识别网络有两个），定位到识别之间的截图代码耗时0.1ms。耗时在100ms以内，其中centernet定位网络耗时所占比例较大。

# 效果对比
一共测试了40张图片（共40*44*2 = 3520个字符），正确检测字符：3509个，多检对象：2个（会多检F字符），漏检对象：1个(漏检L字符)，误检对象：8个(容易把F检成E、Z检成2、8检成B、0检成G、I检成T等)。检测准确率达到3515/3520 = 99.6875%

# 效果分析
centernet定位网络准确度非常高，基本每次都检测到了，后期截图代码也能准确捕捉到机读码。
crnn识别网络比较敏感，不同的字体、视场角、环境等影响较大，由于之前是自己做的假数据做训练，所以测试效果不佳，后来通过调整截图方式及方法对crnn识别准确率有很大程度提高，最终测试可以达到99.6%。
  
效果请参考/data/output文件夹.其中尝试并测试了很多方法,有三部分:1.去除lstm模块,调整网络结构:2.由于网络最后的结构是:transpose->reshape->fc ,transpose层海思平台不支持,想出的办法是去除全链接层,用1 * 1卷积层代替全链接层,将transpose层和reshape层挪到最后,并调换reshape层和transpose层的位置,transpose层当作后处理部分用cpu去实现.(请参考./modify_net文件夹)3.由于densenet网络海思不支持,将主干网络更换成了resnet18.

# 改进的位置

改进之前的网络
维度变化：N * 128 * 1 * 280 ->Transpose -> 280 * 1 * N * 128 -> Reshape -> 280 * N * 128 -> InnerProduct -> 280 * N * 5990 -> CTCGreedyDecoder
<img width="768" alt="lQLPDhrZRxVgNtnNA8PNBgCwY04orhBAFkEBkTsEngAjAA_1536_963" src="https://user-images.githubusercontent.com/57212603/140701049-ffe641c7-bddd-4061-a4ec-20b1d9afba59.png">

改进之后的网络
维度变化：N * 128 * 1 * 280 ->1 * 1conv -> N * 5990 * 1 * 280 -> Reshape -> N * 5990 * 280 -> Transpose -> 280 * N * 5990 -> CTCGreedyDecoder
<img width="768" alt="lQLPDhrZRke9YwTNA8PNBgCwkZ89jOr1SJ8BkTmzLEBYAA_1536_963" src="https://user-images.githubusercontent.com/57212603/140700934-aca9d55a-5d47-4991-b3af-20890fd29f5d.png">

import numpy as np
import matplotlib.pyplot as plt
import copy
import torch


def f(x):
    """
    通过 y=sin(x)^2+x^0.8 获取所需的训练数据。
    :param x:训练数据x, 类型numpy(dim,)
    :return: 训练标签y, 类型numpy(dim,)
    """
    return np.sin(x)*2 + x**0.8


def average_pool(y_train):
    """
    向量平均
    平均汇聚层,实现 f(x)=\frac{1}{n}\sum^n_{i=1}y_i
    :param y_train:输入数据y_train,类型numpy(dim,)
    :return:numpy(公式的输出值 * dim),类型numpy(dim,)
    """
    return np.array([np.sum(y_train)/len(y_train)]*len(y_train))


def softmax(x):
    """
    计算softmax值,将数据归一化到[0,1]
    :param x: 输入数据x, 类型numpy(dim,)
    :return: softmax值, 类型numpy(dim,)
    """
    return np.exp(x)/np.sum(np.exp(x))


def attention_pool(query_x, key, value):
    """
    非参数的注意力汇聚层的实现方法。
    $f(x)=\sum^n_{i=1}softmax(exp(-\frac{1}{2}(x-x_i)^2))y_i$

    :param query_x:查询, 类型numpy(dim,)
    :param key:键,类型numpy(dim,)
    :param value:值,类型numpy(dim,)
    :return attention: 注意力汇聚的加权和,类型numpy(dim)。query_x中的元素,都是该元素通过该计算key的权重,和value的加权和。
    """
    attention = []
    for i in range(len(value)):
        # np.dot() 计算的的是内积  np.dot([0,1,2],[2,3,4])=0*2 + 1*3 + 2*4 = 11
        attention.append(
            np.sum(np.dot(softmax(-(query_x[i] - key)**2/2), value)))
    return attention


def reverse(matrix):
    result = np.zeros_like(matrix)
    row, col = matrix.shape
    for i in range(row):
        result[i] = matrix[row - i - 1]

    return result


def show_heapmap(query_x, x_train):
    """
    计算注意力机制图。
    :param query_x: 查询, 类型numpy(dim,)
    :param x_train: 键, 类型numpy(dim,)
    :return:注意力机制图,类型numpy(dim, dim)
    """
    heapmap = []
    for i in range(len(x_train)):
        # query_x[i] 与每一个x_train(key) 匹配
        heapmap.append(softmax(-(query_x[i] - x_train)**2/2))
    heapmap = reverse(np.array(heapmap))
    return heapmap


class AttentionPoolWithParameter(torch.nn.Module):
    """
    带参数的注意力汇聚实现方法
    """

    def __init__(self):
        super(AttentionPoolWithParameter, self).__init__()
        # 可学习的参数w
        # 1维的tensor
        self.w = torch.nn.Parameter(torch.rand((1,), requires_grad=True))

    # note: 函数参数中的冒号是 输入参数的 类型建议符，告诉程序员希望传入的实参的类型。
    # note: 函数后面跟着的箭头是函数 返回值的 类型建议符，用来说明该函数返回的值是什么类型.

    def forward(self, q: "(1, 50)", k: "(1, 50)", v: "(1, 50)"):
        """
        实现方法。
        :param q: 查询, tensor(1,dim)
        :param k: 键, tensor(1,dim)
        :param v: 值, tensor(1,dim)
        :return: 注意力权重和值的加权和, tensor(1,dim)
        """
        # 通过复制将q的维度，扩展为(dim,dim)，方便计算
        # 化简为矩阵运算是为了并行计算，避免使用 for循环(len(K)){每一个q - K，再相加求和}
        q = q.repeat_interleave(k.shape[1]).reshape(-1, k.shape[1])
        attention = torch.softmax(-((q - k) * self.w)**2/2, dim=1)

        # torch.bmm是矩阵相乘.
        # attention.shape = [50, 50]  attention.unsqueeze(0).shape = [1, 50, 50]
        # v.shape = [1, 50]  v.unsqueeze(-1).shape = [1, 50, 1]
        # torch.bmm(attention.unsqueeze(0), v.unsqueeze(-1)).reshape(1, -1).shape = [1, 50]
        return torch.bmm(attention.unsqueeze(0), v.unsqueeze(-1)).reshape(1, -1)


class CE(torch.nn.Module):
    """
    Cross Entropy Loss
    """

    def __init__(self):
        super(CE, self).__init__()
        self.loss = torch.torch.nn.MSELoss(reduction='none')

    def forward(self, pred, y_true):
        return self.loss(pred, y_true)


def main():
    # ----------------------------------------------------------------------
    # 生成数据
    # ----------------------------------------------------------------------
    # 生成训练数据和标签。
    x_train = np.sort(np.random.rand(50)) * 6
    y_train = f(x_train)  # + np.random.normal(0, 0.5, 50)
    # 生成测试数据和真实标签。
    x_test = np.arange(0, 6.28, 0.12566)
    y_true = f(x_test)
    # 绘制图像
    plt.figure(1)
    l1 = plt.scatter(x_train, y_train, color="r")
    l2, = plt.plot(x_test, y_true, color="b")
    plt.legend(handles=[l1, l2], labels=[
               "train_data", "sin_function"], loc="best")
    plt.savefig("data.png")
    # plt.show()

    # # ----------------------------------------------------------------------
    # # 平均汇聚方法
    # # ----------------------------------------------------------------------
    average_function = average_pool(y_train)

    l3, = plt.plot(x_train, average_function, color="g")
    plt.legend(handles=[l1, l2, l3], labels=["train_data",
               "sin_function", "average_function"], loc="best")
    plt.savefig("average_function.png")
    # plt.show()

    # # ----------------------------------------------------------------------
    # # 非参数的注意力汇聚方法
    # # ----------------------------------------------------------------------
    query_x = copy.deepcopy(x_test)
    attebtion_function = attention_pool(query_x, x_train, y_train)
    l4, = plt.plot(x_train, attebtion_function, color="black")
    plt.legend(handles=[l1, l2, l3, l4], labels=[
               "train_data", "sin_function", "average_function", "attention_pool_no_param"], loc="best")
    plt.savefig("attention_pool_no_param.png")
    # plt.show()
    # 生成注意力机制图。
    heap_map = show_heapmap(x_test, attebtion_function)
    plt.figure(2)
    plt.imshow(heap_map)
    plt.xlabel("x_test")
    plt.ylabel("attebtion")
    plt.savefig("heapmap_no_param.png")
    # plt.show()





    # # # 训练代码：
    # # # ----------------------------------------------------------------------
    # # # 带参数的注意力汇聚方法
    # # # ----------------------------------------------------------------------
    net = AttentionPoolWithParameter()
    optimizer = torch.optim.SGD(net.parameters(), lr=0.5)
    loss = CE()

    x_test = torch.tensor(x_test.astype(np.float32)).reshape(1, -1)
    y_test = torch.tensor(y_true.astype(np.float32)).reshape(1, -1)
    x_train = torch.tensor(x_train.astype(np.float32)).reshape(1, -1)
    y_train = torch.tensor(y_train.astype(np.float32)).reshape(1, -1)

    # 随机初始化 Q
    query_x = np.arange(1, 6, 0.1)
    query_x = torch.tensor(query_x.astype(np.float32)).reshape(1, -1)
    print("query_x.shape: ", query_x.shape)

    # note: model.train()的作用是启用 Batch Normalization 和 Dropout。
    # note: 如果模型中有BN层(Batch Normalization）和Dropout，需要在训练时添加model.train()。
    # note: model.train()是保证BN层能够用到每一批数据的均值和方差。
    # note: 对于Dropout，model.train()是随机取一部分网络连接来训练更新参数。
    net.train()
    for epoch in range(100):
        # print("---   ", epoch, "   ---")

        optimizer.zero_grad()
        y_pred = net(query_x, x_train, y_train)
        l = loss(y_pred, y_train)
        l.sum().backward()
        optimizer.step()
        # print("w :  ", net.w)

        # print(f'epoch {epoch + 1}, loss {float(l.sum()):.6f}')
    # print("w :  ", net.w)
    # note: model.eval()的作用是不启用 Batch Normalization 和 Dropout。
    # note: 如果模型中有BN层(Batch Normalization）和Dropout，在测试时添加model.eval()。
    # note: model.eval()是保证BN层能够用全部训练数据的均值和方差，即测试过程中要保证BN层的均值和方差不变。
    # note: 对于Dropout，model.eval()是利用到了所有网络连接，即不进行随机舍弃神经元。
    net.eval()
    # 推理数据
    x_inference = np.arange(2, 7, 0.1)
    x_inference = torch.tensor(x_inference.astype(np.float32)).reshape(1, -1)

    # note: model.eval()和torch.no_grad()的区别  https://zhuanlan.zhihu.com/p/357075502
    with torch.no_grad():
        print("---  eval  ---")
        print("w :  ", net.w)
        y_pred = net(x_test, x_train, y_train)
        # y_pred = net.forward(x_inference, x_train, y_train)
        plt.figure(1)
        # l5, = plt.plot(x_test.squeeze(), y_pred.squeeze(), color="pink")
        l5 = plt.scatter(x_test.squeeze(), y_pred.squeeze(), color="g")
        # l5 = plt.scatter(x_inference.squeeze(), y_pred.squeeze(), color="g")

        plt.legend(handles=[l1, l2, l3, l4, l5],
                   labels=["train_data", "sin_function", "average_function",
                           "attention_function", "attention_with_params_function"], loc="best")
        plt.savefig("attention_with_params_function.png")

    heap_map = show_heapmap(x_test.squeeze().numpy(), y_pred.squeeze().numpy())
    plt.figure(3)
    plt.imshow(heap_map)
    plt.xlabel("x_test")
    plt.ylabel("y_pred")
    plt.savefig("heapmap_param.png")
    plt.show()


if __name__ == "__main__":
    main()

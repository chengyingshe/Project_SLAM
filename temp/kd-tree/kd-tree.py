import numpy as np

class Node:
    def __init__(self, value, left, right, split):
        self.value = value
        self.left  = left
        self.right = right
        self.split = split # 划分的维度
        
def make_kd_tree(data, k=2):
    if len(data) == 0:
        return None
    data = data.sort(dim=data.split) 			# 对指定维度按照升序排列
    mid = len(data) / 2 # 取出中点
    split = data[mid].split
    node = Node(data[mid], 
      			make_kd_tree(data[0 : mid - 1], k),  			# 左子树
                make_kd_tree(data[mid + 1 : len(data)], k), 	# 右子树
    			(split + 1) % k) # 交替使用各个维度进行划分
    split += 1
    return node


if __name__ == '__main__':
    pass
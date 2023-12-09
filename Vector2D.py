import numpy as np
import math


# 오로지 2D 벡터 값을 나타내기 위한 클래스이다.
# 스칼라곱, 내적, 벡터합, 차, 크기 및 정규화, 그리고 리스트와 튜플로의 데이터 형식 변환 기능이 지원된다.
# 내장 함수로는 두 벡터 사이의 거리를 구하는 distance 함수가 있다.



class Vector2D:
    def __init__(self, _x, _y):
        self.x = _x
        self.y = _y

    def scalarProduct(self, ds):
        return Vector2D(self.x * ds, self.y * ds)

    def __add__(self, other):
        vec = Vector2D(self.x+other.x, self.y+other.y)
        return vec
    
    def __sub__(self, other):
        vec = Vector2D(self.x-other.x, self.y-other.y)
        return vec
    
    def dotProduct(self, other):
        return self.x*other.x + self.y*other.y

    def toList(self):
        return [self.x, self.y]
    
    def toTuple(self):
        return (self.x, self.y)
    
    def magnitude(self):
        length = (self.x ** 2 + self.y ** 2) ** 0.5
        return length

    def normalize(self):
        length = self.magnitude()
        if length == 0:
            return Vector2D(0, 0)
        return Vector2D(self.x / length, self.y / length)
    
def distance(vec1, vec2): 
    lenVec = (vec2[0] - vec1[0], vec2[1]-vec1[1])
    magnitude = pow(lenVec[0],2) + pow(lenVec[1],2)
    return math.sqrt(magnitude)
from Vector2D import *
from math import *


wallMass = 1000000 # 벽의 질량을 나타내기 위한 변수이다. 현 시점에서 활용 가치는 없다.


class Transform:
    # 위치, 스케일, 회전 정보를 가진 오브젝트를 생성한다. 

    def __init__(self, _px, _py, _points, name="obj"):
         # 초기값으로 위치, 경계 포인트를 입력한다

        self.position = Vector2D(_px, _py)
        self.velocity = Vector2D(0.0,0.0)
        self.acceleration = Vector2D(0.0,0.0)

        self.name = name

        self.scale = Vector2D(1.0, 1.0)
        self.angle = 0

        self.points = _points

    def Translation(self, dx, dy, mode = "+"):
        # 지정된 값만큼 위치를 이동시킨다.

        if mode == '+':
            self.position = self.position + Vector2D(dx,dy)
        elif mode == '=':
            self.position = Vector2D(dx,dy)

    def Move(self, dt):
        # 지정된 속도로 시간 변화량 만큼 이동시킨다.
        self.velocity = self.velocity + self.acceleration.scalarProduct(dt)
        self.position = self.position + self.velocity.scalarProduct(dt)

    def Scaling(self, dx, dy):
        # 오브젝트의 스케일 값을 변경한다.

        self.scale.x = dx
        self.scale.y = dy

    def Rotation(self, da, mode = '+'):
        # 오브젝트를 일정 값만큼 회전시킨다.

        if mode == '+':
            self.angle = self.angle + da
        elif mode == '=':
            self.angle = da

    def transform(self):
        # 경계 포인트를 위치, 스케일, 회전 값에 맞게 조정하고 리스트를 반환한다.
        # 화면에 그려지는 오브젝트는 이 함수로 반환된 리스트를 바탕으로 그려진다.

        transformed_points = []

        for p in self.points:

            scale_points = Vector2D(p.x * self.scale.x, p.y * self.scale.y)

            rotated_x = scale_points.x * cos(radians(self.angle)) - scale_points.y * sin(radians(self.angle))
            rotated_y = scale_points.x * sin(radians(self.angle)) + scale_points.y * cos(radians(self.angle))

            rotate_points = Vector2D(rotated_x, rotated_y)

            translate_point = rotate_points + self.position

            transformed_points.append((int(translate_point.x), int(translate_point.y)))

        return transformed_points

class CollisionDetection(Transform):
    # 오브젝트의 충돌을 감지할 수 있는 클래스이다.
    # 초기값은 Transform과 동일하며 콜라이더의 크기는 경계 포인트와 동일하다.
    # 오브젝트 생성 시 사용한다.



    
    def __init__(self, _x, _y, _points, mass = 1, name="obj"):
        # Transform을 베이스 클래스로 하므로 동일한 멤버를 보유하고 있다.
        # 다만 이 클래스는 충돌 범위를 다루고 있으므로 이에 대한 멤버변수가 따로 존재한다.

        super().__init__(_x,_y,_points, name)

        self.colliderPosition = Vector2D(0,0)
        self.colliderScale = self.scale
        self.colliderAngle = self.angle

        self.collider = self.points
        self.mass = mass

        self.kindOfCollider = "POLYGON"
        self.collisionTimer = 0
        self.latelyCollidedObject = None

    def setPosition(self, dx, dy, mode = "="):
        # Collider의 위치를 변경한다.
        # + 모드인 경우 포지션을 Translation하고 = 모드인 경우 포지션을 재설정한다.
        # Collider의 위치는 오브젝트의 model coordinate 기준이다.
        
        if mode == '+':
            self.colliderPosition = self.colliderPosition + Vector2D(dx,dy)
        elif mode == '=':
            self.colliderPosition = Vector2D(dx,dy)

    def setScale(self, sx, sy):
        # Collider의 스케일 값을 재설정한다.

        self.colliderScale = Vector2D(sx, sy)

    def setAngle(self, da, mode = '='):
        # Collder의 회전 정보를 갱신한다.
        # + 모드인 경우 일정값만큼 회전시키고 = 모드인 경우 회전값을 재설정한다.

        if mode == '+':
            self.colliderAngle = self.colliderAngle + da
        elif mode == '=':
            self.colliderAngle = da

    def getCollider(self):
        # 실제 충돌 범위를 반환한다.
        # OBB 함수 실행 시 사용한다.

        transformed_collider_points = []

        for p in self.collider:

            scale_points = Vector2D(p.x * self.colliderScale.x * self.scale.x, p.y * self.colliderScale.y* self.scale.y)

            rotated_x = scale_points.x * cos(radians(self.colliderAngle+self.angle)) - scale_points.y * sin(radians(self.colliderAngle+self.angle))
            rotated_y = scale_points.x * sin(radians(self.colliderAngle+self.angle)) + scale_points.y * cos(radians(self.colliderAngle+self.angle))

            rotate_points = Vector2D(rotated_x, rotated_y)

            translate_point = rotate_points + self.colliderPosition + self.position

            transformed_collider_points.append((int(translate_point.x), int(translate_point.y)))

        return transformed_collider_points
    
    def Move(self, dt):
        self.collisionTimer -= dt
        if self.collisionTimer <= 0:
            self.collisionTimer = 0 
            self.latelyCollidedObject = None

        # 지정된 속도로 시간 변화량 만큼 이동시킨다.
        self.velocity = self.velocity + self.acceleration.scalarProduct(dt)
        self.position = self.position + self.velocity.scalarProduct(dt)







# ------------------------------ OBB 알고리즘을 이용한 충돌 감지 코드이다. ------------------------------ #

    def OBB(self, other):
        # 다른 오브젝트와의 OBB 충돌 감지를 수행한다.
        # 충돌이 감지될 경우 true, 미감지될 경우 false를 반환한다.
        # 충돌하는 객체의 collider가 polygon이고 convex 형태인 경우 적용한다.

        myCollider = self.getCollider()
        youCollider = other.getCollider()

        for axis in self.getAxes() + other.getAxes():
            if not self.overlapOnAxis(myCollider, youCollider, axis):
                return False
        return True

    def getAxes(self):
        # 오브젝트의 모든 변에 대한 법선 벡터를 정규화하여 반환함.

        axes = []
        myCollider = self.getCollider()

        for i in range(len(myCollider)):
            edge = Vector2D(myCollider[i][0] - myCollider[i-1][0], myCollider[i][1] - myCollider[i-1][1])
            axes.append(Vector2D(-edge.y, edge.x).normalize())

        return axes
    
    def overlapOnAxis(self, points1, points2, axis):
        # 파라미터로 주어진 axis는 두 오브젝트의 모든 법선 벡터임.
        # 

        def projectPointsOnAxis(points):
            minProjection = float('inf')
            maxProjection = float('-inf')

            for point in points:
                projection = point[0] * axis.x + point[1] * axis.y
                minProjection = min(minProjection, projection)
                maxProjection = max(maxProjection, projection)

            return minProjection, maxProjection
        min1, max1 = projectPointsOnAxis(points1)
        min2, max2 = projectPointsOnAxis(points2)

        return not (max1 < min2 or max2 < min1)






# ----------------------------------- Raycast 방식을 이용한 충돌 감지 코드이다. --------------------------------- #

    def line_segment(self, other):
        # 레이 캐스팅을 이용하여 충돌을 감지한다.
        # 충돌 발생 시 해당 좌표를 반환한다.
        # 충돌이 없다면 None을 반환한다.
        if self.latelyCollidedObject == other:
            return None

        myCollider = self.getCollider()
        yourCollider = other.getCollider()

        for i in range(len(myCollider)):
            for j in range(len(yourCollider)):
                #myLine = myCollider[i] - myCollider[i-1]
                #yourLine = yourCollider[i] - yourCollider[i-1]

                collisionPoint = getCrossPoint(myCollider[i], myCollider[i-1], yourCollider[j], yourCollider[j-1])
                if collisionPoint:
                    self.latelyCollidedObject = other
                    other.latelyCollidedObject = self

                    self.collisionTimer = 0.5
                    other.collisionTimer = 0.5
                    return (myCollider[i], myCollider[i-1], yourCollider[j], yourCollider[j-1])
                
                
        return None
    



# ----------------------------------------- 범위 내 객체가 있는지 파악하는 코드이다. ------------------------------------- # 
    
    def overlapCircleArea(self, radius, obj):
        # 내 위치와 특정 오브젝트의 콜라이더 사이의 최소 거리가 radius내에 있다면
        # 해당 오브젝트는 정해진 범위 내에 있다고 가정

        # 해당 함수는 충돌의 최적화를 위해 활용

        minDistance = float('inf')
        
        objCollider = obj.getCollider()
        for p in objCollider:
            minDistance = min(minDistance, distance(self.position.toTuple(), p))
        
        return minDistance < radius

    def overlapCircularArea(self, radius, angle, obj):
        # 내 위치와 특정 오브젝트의 콜라이더 사이의 최소 거리가 radius내에 있다면
        # 해당 오브젝트는 정해진 범위 내에 있다고 가정

        minDistance = float('inf')
        nDirectionVector = self.velocity.normalize()
        nPositionVector = ()

        

        objCollider = obj.getCollider()
        for p in objCollider:
            minDistance = min(minDistance, distance(self.position.toTuple(), p))
            nPositionVector = (obj.position - self.position).normalize()

        arccos = acos(nDirectionVector.dotProduct(nPositionVector)) # => only cosine value between two vectors

        return (minDistance < radius and arccos < radians(angle))
        

class BoxCollider(CollisionDetection):

    # CollisionDetection로 객체 생성 시 Collider 좌표를 직접 지정해야 하므로
    # 이를 방지하기 위해 size 변수 하나로 즉시 Box형 Collider를 만들 수 있도록 했다.

    def __init__(self, _x, _y, size=20, mass = 1, name="obj"):

        points = [Vector2D(size/2, size/2),Vector2D(-size/2, size/2),Vector2D(-size/2, -size/2),Vector2D(size/2, -size/2)]

        super().__init__(_x, _y, points, mass, name)
        self.collider = self.points
        self.mass = mass
        self.kindOfCollider = "BOX"



# -------------------------------------------- 아래는 컴포넌트 클래스들의 상호작용을 위한 내장 함수이다. ---------------------------------------- #




    
# LineSegment에서 교점을 얻기 위한 함수이다.
def getCrossPoint(u1, u2, v1, v2):
    # 파라미터는 tuple임
    d1, d2 = distance(u1, u2), distance(v1, v2)
    try:
        x, y = intersection(u1,u2,v1,v2)
        if (distance(u1, (x,y)) <= d1 and
                distance(u2, (x,y)) <= d1 and
                distance(v1, (x,y)) <= d2 and
                distance(v2, (x,y)) <= d2):
            return True
        else:
            return False
    except np.linalg.linalg.LinAlgError:
        return False


# 위 함수에서 각 선분이 이루는 직선의 계수를 구하는 함수이다.
def getCoefficient(v1, v2):
    x1, y1 = v1
    x2, y2 = v2

    a=y2-y1
    b=x1-x2
    c=x1*y2-y1*x2
    return a,b,c


# 직선의 계수를 통해 교점을 구하는 함수이다.
def intersection(u1 ,u2, v1, v2):
    a1,b1,c1 = getCoefficient(u1,u2)
    a2,b2,c2 = getCoefficient(v1,v2)
    #print(a1,b1,c1)
    #print(a2,b2,c2)
    m = np.array(((a1,b1),(a2,b2)))
    c = np.array((c1,c2))
    return np.linalg.solve(m,c)



# 두 객체 간 충돌 발생 시 비탄성 충돌에 의한 속도 변화를 만드는 함수이다.
# 만약 함수 내부 elasticity의 값을 1로 만들면 완전 탄성 충돌이다.
# 하지만 완전 탄성 충돌은 오류가 발생하므로 현 시점에서 사용하지 않는다.
def impulsiveForceForAngle(obj1, obj2):


    collision_vector = obj1.position - obj2.position
    collision_angle = math.atan2(collision_vector.y, collision_vector.x)
    relative_velocity = obj1.velocity - obj2.velocity
    relative_speed = relative_velocity.magnitude()

    angle_diff = math.atan2(relative_velocity.y, relative_velocity.x) - collision_angle
    elasticity = 0

    new_relative_speed = elasticity * relative_speed

    new_relative_velocity = Vector2D(math.cos(collision_angle + angle_diff) * new_relative_speed,
                                    math.sin(collision_angle + angle_diff) * new_relative_speed)

    obj1_new_velocity = obj2.velocity + new_relative_velocity
    obj2_new_velocity = obj1.velocity - new_relative_velocity

    return obj1_new_velocity, obj2_new_velocity


# 벽과 충돌 시 수행하는 함수이다.
# 벽은 고정되며 변형되지 않기 때문에 질량은 무한대라 가정하고 새로운 공식을 적용한다.
# LineSegment를 통해 충돌한 콜라이더의 정보를 바탕으로 법선 벡터를 얻고 이를 이용해 새로운 속도를 만든다.
def impulsiveForceWithWall(obj, collider):
    
    collisionVector = Vector2D(collider[0][0] - collider[1][0], collider[0][1] - collider[1][1])
    normalVector = Vector2D(-collisionVector.y, collisionVector.x).normalize()

    new_velocity = obj.velocity - normalVector.scalarProduct((obj.velocity.scalarProduct(2).dotProduct(normalVector)))

    obj.position = obj.position + new_velocity.scalarProduct(0.5)
    return new_velocity








# --------------------------------------------- 아래는 논리적, 기능적 이유로 사용하지 않지만 모듈에 포함은 시켜둔 함수이다. ------------------------------ #


# OBB를 다루기 위해 만든 함수이나 사용하지 않는다. 
# OBB를 이용해 충돌 감지 시 속도 변화를 주는 기능이다.
# 충돌 감지 시 두 객체 사이의 중심 위치를 기준으로 대칭시켜 속도의 변화를 준다.   
def impulsiveForce(obj1, obj2):


        
    massSum = obj1.mass + obj2.mass
    massSub = obj1.mass - obj2.mass

    collisionCoordinate = (obj1.position - obj2.position)
    m = collisionCoordinate.y / collisionCoordinate.x

        

    vel1 = obj1.velocity.scalarProduct(massSub/massSum) + obj2.velocity.scalarProduct(2*obj2.mass/massSum)
    vel2 = obj2.velocity.scalarProduct(-massSub/massSum) + obj1.velocity.scalarProduct(2*obj1.mass/massSum)
    if obj1.mass == wallMass or obj2.mass == wallMass:
        return vel1, vel2
    
    new1 = reflectToSlope(vel1, m)
    new2 = reflectToSlope(vel2,m)
    print("In Slope m : ",m)
    print("(",vel1.x,",",vel1.y,")", " vs ", "(",new1.x,",",new1.y,")")
    print("(",vel2.x,",",vel2.y,")", " vs ", "(",new2.x,",",new2.y,")")
    
   
    return new1, new2 
    

# 기울기 m을 가지는 직선에 대해 특정 벡터를 대칭시키는 함수이다.
def reflectToSlope(vec, m):
    
    mp = pow(m,2)
    leftDown = (mp-1) / (mp+1)
    rightDown = (2*mp) / (mp+1)

    a = -leftDown * vec.x + rightDown * vec.y
    b = rightDown * vec.x + leftDown * vec.y

    return Vector2D(a,b)

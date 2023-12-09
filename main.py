from Component import *
from Vector2D import *
import pygame
import sys

player_box_size = 15
width = 1280
height = 960
speed = 3

def handle_input(player, keys, dt):
    global speed
    # 키 입력을 처리하여 이동 및 회전을 조절하는 기능이다.
    # 플레이어의 위치 변경과 위치 초기화 등의 기능을 담당한다.

    # 방향키 입력 시 해당 방향의 가속도가 정의된다.
    # 키 입력 중단 시 해당 방향의 가속도가 0이 되고 서서히 속도가 감소한다.
    
    if keys[pygame.K_LEFT]:
        player.acceleration.x = -speed
    elif keys[pygame.K_RIGHT]:
        player.acceleration.x = speed
    else:
        player.acceleration.x = 0
        player.velocity.x = round(player.velocity.x*0.95, 1)

    if keys[pygame.K_UP]:   
        player.acceleration.y = -speed
    elif keys[pygame.K_DOWN]:
        player.acceleration.y = speed
    else:
        player.acceleration.y = 0
        player.velocity.y = round(player.velocity.y*0.95, 1)


    # 스페이스 키를 통해 회전시킨다.
    if keys[pygame.K_SPACE]:
        player.Rotation(1)


    # 좌쉬프트 키를 이용해 일부 정보를 초기화한다.
    if keys[pygame.K_LSHIFT]:
        player.position = Vector2D(width/2, width/2)
        player.velocity = Vector2D(0,0)
        player.acceleration = Vector2D(0,0)


    # 해당 함수는 항상 실행되므로 Move함수를 사용해 지속적인 움직임을 실행한다.    
    player.Move(dt)


def main():

    # 전역변수 및 지역변수 정의
    global width, height
    global player_box_size
    pbs = player_box_size

    detectionDistance = 100
    detectionAngle = 30

    collisionType = 1
    pygame.init()




    # 플레이어의 초기 위치 및 Collider 위치 정의
    initial_x, initial_y = width/2, height/2
    initial_points = [Vector2D(pbs, pbs), Vector2D(-pbs, pbs), Vector2D(-pbs, -pbs), Vector2D(pbs, -pbs)]



    # 플레이어 객체를 생성한다.
    player = BoxCollider(initial_x, initial_y, pbs*2, 2)
    player.colliderScale = Vector2D(1.5, 1.5)



    # 적 객체를 생성한다.
    e_points = [Vector2D(pbs, pbs), Vector2D(-pbs, pbs), Vector2D(-pbs, -pbs), Vector2D(pbs, -pbs)]
    enemy_list = [
                  BoxCollider(200, 200, pbs*2, 2),
                  BoxCollider(600,600,pbs*2, 2),
                  BoxCollider(200,600,pbs*2, 2),
                  BoxCollider(600,200,pbs*2, 2),
                  BoxCollider(300,300,pbs*2, 2),
                  BoxCollider(100,100,pbs*2, 2)
                  ]
    enemy_list[2].velocity = Vector2D(10, -10)



    # 벽 객체를 생성한다.
    # 벽에 대해서는 다른 오브젝트와 다른 로직을 적용한다.
    # ex) 충돌 발생 시 속도 처리에 대한 로직
    walls = [CollisionDetection(width/2, 0, [Vector2D(-width/2,10),Vector2D(width/2,10),Vector2D(width/2,-10),Vector2D(-width/2,-10)], wallMass), 
             CollisionDetection(width/2, height, [Vector2D(width/2,-10),Vector2D(-width/2,-10),Vector2D(-width/2,10),Vector2D(width/2,10)], wallMass), 
             CollisionDetection(0, height/2, [Vector2D(10,-height/2),Vector2D(10,height/2),Vector2D(-10,height/2),Vector2D(-10,-height/2)], wallMass),
             CollisionDetection(width, height/2, [Vector2D(-10,-height/2),Vector2D(-10, height/2),Vector2D(10,height/2),Vector2D(10,-height/2)], wallMass)] 

    # 화면 및 틱 구성
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()
    wCollisiontime = 0

    current_time = pygame.time.get_ticks()
    elapsed_time = 0


    # 매 틱마다 업데이트하는 무한 루프이다.
    # 강제 종료 전까지 계속 실행한다.
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        
        # 매 틱마다 경과된 시간을 얻는다.
        # 움직임, 상태 갱신, 충돌 감지 등 여러 기능에서 활용한다.
        elapsed_time = pygame.time.get_ticks() - current_time
        wCollisiontime += elapsed_time/1000

        keys = pygame.key.get_pressed()
        

        # 플레이어 및 적 객체의 움직임을 갱신한다.
        handle_input(player, keys, elapsed_time/100)
        for e in enemy_list:
            e.Move(elapsed_time/50)



        # 화면 초기화
        screen.fill((255, 255, 255))



        # collisionType을 0으로 설정하면 OBB를 바탕으로 충돌 감지를 수행한다.
        # 하지만 충돌 감지는 OBB로 수행하지 않고 더 간단한 Raycasting으로 수행하여 최적화한다.
        if collisionType == 0:
            if time > 0.15:
                time = 0
                for e in enemy_list:
                    if player.OBB(e):
                        #print("충돌!!!!!!!!  with " + e.name)
                        player.velocity, e.velocity = impulsiveForceForAngle(player, e)
                        #print(player.velocity.x,player.velocity.y , e.velocity.x, e.velocity.y)
                    
                    for w in walls:
                        if w.OBB(e):
                            a, e.velocity = impulsiveForceForAngle(w,e)

        
        # CollisionType이 1인 경우 Raycast 방식으로 충돌 감지를 수행한다.
        # 충돌 감지 주기를 틱보다 길게하여 최적화를 수행했다.
        elif collisionType == 1:
            if wCollisiontime > 0.01:
                
                for e in enemy_list:
                    #print(c_point)
                    
                    for w in walls:
                        w_points = w.line_segment(e)
                        if w_points:
                            e.velocity = impulsiveForceWithWall(e,w_points)
                        
                        w_points = w.line_segment(player)
                        if w_points:
                            player.velocity = impulsiveForceWithWall(player,w_points)

                    if player.overlapCircleArea(pbs*2, e):
                        if player.line_segment(e):
                            player.velocity, e.velocity = impulsiveForceForAngle(player, e)
                    for ee in enemy_list:
                        if e!=ee and ee.overlapCircleArea(pbs*2,e):
                            if ee.line_segment(e):
                                ee.velocity, e.velocity = impulsiveForceForAngle(ee, e)   
                wCollisiontime = 0
                    







# ---------------------------------- 아래는 스크린 업데이트를 위한 함수 모음이다. -------------------------------- #
# 폴리곤 드로잉과 라인 드로잉, 스크린 갱신 등의 기능이 포함되어 있다.
# 라인 드로잉을 위한 변환 과정이 포함되어 있다.

    

    # 속도 방향에 따라 전방 60도 내의 오브젝트를 인식하는데 시각적 도움을 주는 라인을 생성하기 위한 코드이다. 

        v_points = [Vector2D(0, 0), player.velocity.normalize().scalarProduct(detectionDistance)]
        #pygame.draw.aaline(screen, (0, 0, 0), (v_points[0] + player.position).toTuple(), (v_points[1] + player.position).toTuple(), 2)



        # 전방 인식을 위한 임시 코드이다.
        # overlapArea 기능은 언제든 사용 가능한 메소드이다.
        color = (0, 0, 0)
        for e in enemy_list:
            if player.overlapCircularArea(detectionDistance, detectionAngle, e):
                #print("전방 60도 내 물체 감지")
                color = (255, 0, 0)




        # 회전 변환 공식을 이용하여 좌우로 30도 회전
        angle_radians_positive = math.radians(30)
        angle_radians_negative = math.radians(-30)

        cos_positive = math.cos(angle_radians_positive)
        sin_positive = math.sin(angle_radians_positive)

        cos_negative = math.cos(angle_radians_negative)
        sin_negative = math.sin(angle_radians_negative)

                # 회전 변환
        p1 = Vector2D(
            v_points[1].x * cos_positive - v_points[1].y * sin_positive,
            v_points[1].x * sin_positive + v_points[1].y * cos_positive
        ) + player.position

        p2 = Vector2D(
             v_points[1].x * cos_negative - v_points[1].y * sin_negative,
            v_points[1].x * sin_negative + v_points[1].y * cos_negative
        ) + player.position

        pygame.draw.aaline(screen, color, player.position.toTuple(), p1.toTuple())
        pygame.draw.aaline(screen, color, player.position.toTuple(), p2.toTuple())





    # 모든 폴리곤 객체들을 화면에 그리기 위한 기능이다.
    # 벽의 최적화 처리를 위한 임시 코드가 포함되었다.
            
        initial_points = player.transform()
        pygame.draw.polygon(screen, (255, 0, 0), initial_points)
        
        for w in walls:
            w.Move(elapsed_time/100)
            w_points = w.transform()
            pygame.draw.polygon(screen, (0,0,255), w_points)
        for e in enemy_list:
            e_points = e.transform()
            pygame.draw.polygon(screen, (0,255,0), e_points)

        

        
        current_time = pygame.time.get_ticks()
        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()
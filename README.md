# 미니드론 자율주행 경진대회 A리그 텔로토비팀!🚀[![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)]

![드론관점주행](/readmeFile/드론관점주행.gif) 

## 1. 대회 진행 전략

 1. 드론 이륙

- **이미지 전처리**
 2. 텔로 드론의 영상을 받아와 [Snapshot](https://kr.mathworks.com/help/supportpkg/ryzeio/ref/snapshot.html) 함수를 이용하여 사진 영상을 가져옴
 3. RGB 색 공간에서 HSV 색 공간으로 변경함
 4. 링 통과를 위해 HSV 색 공간을 통하여 초록색을 찾고 링만 인식하기 위해 이진화를 실행함
 5. 회전점인 <span style="color:red">빨간색</span>과 착지점인 <span style="color:blue">파란색</span>도 동일하게 이진화를 실행함

- **드론 제어**

 6. 링이 왼쪽에 있는지 오른쪽에 있는지 판단하기 위해 영상을 반으로 나누어 각각 픽셀 수를 더하여 왼쪽이 많으면 왼쪽으로 ROLL 제어 이동하고, 오른쪽이 많으면 오른쪽으로 ROLL 제어 이동함
그리고 양쪽 값이 같다고 판단되면 PITCH 제어로 위로 올라가 링 구멍을 찾음
 7. 이렇게 링을 찾은 뒤 링 구멍을 픽셀 좌표 찾는 방법을 사용하여 중심점을 찾음
 8. 위에서 받은 Snapshot 영상의 중심점과 링 구멍 중심점의 차이를 계산 후 **X점의 차이가 ±49이면 좌우로 보정**하고, **Y점 차이가 ±30이면 상하로 보정**한 뒤 **전진** 함
 9. 링 통과를 한 후 빨간색 원을 발견하게 되는데, 이 때 픽셀 수를 합하여 **500이 넘으면 좌회전**을 하고, **500이 넘지 못하면 후진**을 하여 다시 찾음
 10. 마지막으로 파란색 원을 발견하게 되면 위와 같이 **500이 넘으면 착지**하고, 아니면 **후진**하여 다시 찾음

<br></br>
## 2. 알고리즘 설명
<br></br>
## 3. 소스코드 설명
```matlab
clear;
drone=ryze(); 
cam=camera(drone);
originCenter=[480 200];
count=0;
max=0;
takeoff(drone);
preview(cam);

%초록색 링 인식
while 1
    frame=snapshot(cam);%화면 캡쳐
    hsv = rgb2hsv(frame);%rgb값을 hsv값으로 변환
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    v = hsv(:,:,3);
    green=(0.35<h)&(h<0.45)&(0.65<s)&(s<0.95);%색에 대해 조건이 맞으면 1로, 조건에 안맞으면 0으로 반환됨
    %링이 왼쪽 or 오른쪽에 있는지 판단 후 이동 및 없으면 위로 이동
    if sum(imcrop(bw2,[0 0 480 720]),'all')-sum(imcrop(bw2,[480 0 960 720]),'all')>10000 %x축을 기준으로 반으로 잘라 좌우 비교
        moveleft(drone,'distance',0.3); %왼쪽이 많으면 왼쪽으로 이동
    elseif sum(imcrop(bw2,[480 0 960 720]),'all')-sum(imcrop(bw2,[0 0 480 720]),'all')>10000
        moveright(drone,'distance',0.3); %오른쪽이 많으면 오른쪽으로 이동
    else
        moveup(drone,'distance',0.3); %비슷하다고 인식하면 위로 이동(링이 보이지 않은 경우)
    end
    %imfill의 한계를 극복하기 위해 구멍을 만들어주기 위해 첫행을 1로 변환
    for i=1:960
    green(1,i)=1;
    end
    bw2 = imfill(green,'holes'); %구멍으로 인식된 부분을 채움
    %채워진 이미지와 채워지지 않은 이미지를 비교하여 값이 같으면 0으로 달라지면 1로 변환(이로써 링의 구멍을 찾음)
    for x=1:720
        for y=1:960
            if green(x,y)==bw2(x,y)
                bw2(x,y)=0;
            end
        end
    end
    
    %링이 인식되면 반복문 탈출
    if sum(bw2,'all')>1000
        break;
    end
end

%중점 찾기
while 1
    frame=snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    v = hsv(:,:,3);
    green= (0.35<h)&(h<0.45)&(0.65<s)&(s<0.95);
    %imfill의 한계를 극복하기 위해 구멍을 만들어주기 위해 첫행을 1로 변환
    for i=1:960
        green(1,i)=1;
    end
    bw2 = imfill(green,'holes'); %구멍으로 인식된 부분을 채움
    %채워진 이미지와 채워지지 않은 이미지를 비교하여 값이 같으면 0으로 달라지면 1로 변환(이로써 링의 구멍을 찾음)
    for x=1:720
        for y=1:960
            if green(x,y)==bw2(x,y)
                bw2(x,y)=0;
            end
        end
    end
    %다음 함수를 이용하여 중점과 크기가 가장 큰 부분을 찾음(잡음 부분들의 중심을 제거)
    stats = regionprops('table',bw2, 'Centroid', 'MinorAxisLength');
    z=stats.MinorAxisLength;
    max=0;
    y=stats.Centroid;
    %MinorAxisLength의 가장 큰 값을 찾고 그에 대한 중점을 찾아 firstCenter에 대입
    for i=1:size(stats)
        if z(i,1)>=max
            max=z(i,1);
            firstCenter(1,1)=round(y(i,1));
            firstCenter(1,2)=round(y(i,2));
        end
    end
    
    %firstCenter와 처음 설정한 이상중점을 비교
    %x축 오차 49, y축 오차 30정도를 주고 중점으로 이동
    if firstCenter(1,1)-originCenter(1,1)>=49
        moveright(drone,'Distance',0.2);
        disp("right");
    elseif firstCenter(1,1)-originCenter(1,1)<=-49
        moveleft(drone,'Distance',0.2);
        disp("left");
    end
    if firstCenter(1,2)-originCenter(1,2)>=30                         
        movedown(drone,'Distance',0.2);
        disp("down");
    elseif firstCenter(1,2)-originCenter(1,2)<=-30
        moveup(drone,'Distance',0.2);
        disp("up");
    end
%중점으로 인식하면 전진
    if firstCenter(1,2)-originCenter(1,2)<30 && firstCenter(1,2)-originCenter(1,2)>-30 && firstCenter(1,1)-originCenter(1,1)<49 && firstCenter(1,1)-originCenter(1,1)>-49
        moveforward(drone,'Distance',2.7);
        break;
    end
end

%빨간색 점 인식
while 1
frame=snapshot(cam);
hsv = rgb2hsv(frame);
h = hsv(:,:,1);
s = hsv(:,:,2);
v = hsv(:,:,3);
red= ((0.80<h)&(h<1)|(0<h)&(h<0.1))&(0.6<s)&(s<0.9)&(0.2<v)&(v<0.5);
%빨간색의 값이 400픽셀이 넘으면 인식했다고 파악 
%인식하면 90도 회전 후 반복문 탈출
if sum(red,'all')>400
    turn(drone,deg2rad(-90))
    break;
end
end
%기준을 1단계 높이의 가운데로 잡아 기준으로 이동
moveforward(drone,'Distance',0.7);
moveright(drone,'Distance',0.3);
while 1
    if readHeight(drone)<0.8
        break;
    else
        movedown(drone,'distance',0.3);
    end
end
    clearvars max %max변수 초기화

%파란색 점 인식
while 1
frame=snapshot(cam);
hsv = rgb2hsv(frame);
h = hsv(:,:,1);
s = hsv(:,:,2);
v = hsv(:,:,3);
blue= (0.55<h)&(h<0.7)&(0.5<s)&(s<0.7)&(0.2<v)&(v<0.5);
%파란색의 값이 400픽셀이 넘으면 인식했다고 파악 
%인식하면 착륙 후 반복문 탈출
if sum(blue,'all')>400
    land(drone);
    disp("goal")
    break;
end
end

```

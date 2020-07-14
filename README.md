# 미니드론 자율주행 경진대회 A리그 텔로토비팀!🚀
[![TelloToby](https://img.shields.io/badge/Tello-Toby-blue.svg)](https://github.com/jungdeayun/Aleague_tellotoby) 

![드론관점주행](/readmeFile/drone.gif) 
<br></br>
## 1. 대회 진행 전략
![전략](/readmeFile/전략.png)
### 저희 팀의 전략은 상하좌우의 픽셀 수를 비교해 이동하여 초록색 링을 인식하게 하여 구멍을 찾은 뒤, 드론 영상 크기의 중심점과 드론 영상의 링 구멍의 중심점의 차이를 계산하여 통과하도록 합니다. 빨간색 점과 파란색 점도 똑같이 픽셀 수를 이용해 90도 회전 또는 착지하도록 합니다.

<br></br>
## 2. 알고리즘 설명
![알고리즘도](/readmeFile/알고리즘.png)

 1. 드론 이륙

- **이미지 전처리**
 2. 텔로 드론의 영상을 받아와 [Snapshot](https://kr.mathworks.com/help/supportpkg/ryzeio/ref/snapshot.html) 함수를 이용하여 사진 영상을 가져옴
 3. RGB 색 공간에서 HSV 색 공간으로 변경함
 4. 링을 찾기 위해 HSV 색 공간을 통하여 초록색을 찾고 이진화를 실행함
 5. 회전점인 빨간색과 착지점인 파란색도 동일하게 이진화를 실행함
 6. [imfill](https://kr.mathworks.com/help/images/ref/imfill.html)을 사용하기 위해 첫 행과 마지막 행을 1로 바꿈
 7. imfill한 이미지와 하지 않은 이미지를 비교해 바뀐 부분을 1으로 바뀌지 않은 부분을 0으로 하여 링 인식

- **드론 제어**

 8. 링이 왼쪽에 있는지 오른쪽에 있는지 판단하기 위해 영상을 반으로 나누어 각각 픽셀 수를 더하여 왼쪽이 많으면 왼쪽으로 ROLL 제어 이동하고, 오른쪽이 많으면 오른쪽으로 ROLL 제어 이동함
그리고 양쪽 값이 같다고 판단되면 THROTTLE 제어로 위로 올라가 링 구멍을 찾음
 9. 이렇게 링 을 찾은 뒤 [regionprops](https://kr.mathworks.com/help/images/ref/regionprops.html)함수를 사용하여 중심점을 찾음
 10. 위에서 받은 Snapshot 영상의 중심점과 링 구멍 중심점의 차이를 계산 후 **X값의 차이가 40이면 좌우로 보정**하고, **Y값의 차이가 30이면 상하로 보정**한 뒤 **전진** 함
 11. 링 통과를 한 후 빨간색 원을 발견하게 되는데, 이 때 픽셀 수를 합하여 **400이 넘으면 좌회전**을 하고, **400이 넘지 못하면 후진**을 하여 다시 찾음
 12. 마지막으로 파란색 원을 발견하게 되면 위와 같이 **300이 넘으면 착지**하고, 아니면 **후진**하여 다시 찾음

<br></br>
## 3. 소스코드 설명

- **드론 설정 및 takeoff**
```matlab
clear;
drone=ryze(); 
cam=camera(drone);
originCenter=[480 200; 480 170; 480 170];
count=0;
max=0;
takeoff(drone);
preview(cam);
```
- **구멍 찾기**
```matlab
%총 3단계로 설정
for level=1:3
    %구멍 찾기
    while 1
        %초록색에 대한 HSV값 설정 및 이진화
        frame=snapshot(cam);
        hsv = rgb2hsv(frame);
        h = hsv(:,:,1);
        s = hsv(:,:,2);
        v = hsv(:,:,3);
        green=(0.35<h)&(h<0.45)&(0.65<s)&(s<0.95);
        %첫 행을 1로 변환
        for i=1:960
        green(1,i)=1;
        end
        %마지막 행을 1로 
        for i=1:960
        green(720,i)=1;
        end
        %구멍을 채움
        bw2 = imfill(green,'holes');
        %구멍을 채우기 전과 후를 비교하여 값이 일정하면 0, 변했으면 1로 변환
        for x=1:720
            for y=1:960
                if green(x,y)==bw2(x,y)
                    bw2(x,y)=0;
                end
            end
        end
        %변환한 이미지의 픽셀 수가 1000이상이면 구멍을 인식했다고 파악
        %1000이하이면 상승하여 전 과정을 다시 반복
        if sum(bw2,'all')>1000
            break;
        else
            %화면의 좌우를 비교
            if sum(imcrop(green,[0 0 480 720]),'all')-sum(imcrop(green,[480 0 960 720]),'all')>10000
                moveleft(drone,'distance',0.5,'speed',1);
            elseif sum(imcrop(green,[480 0 960 720]),'all')-sum(imcrop(green,[0 0 480 720]),'all')>10000
                moveright(drone,'distance',0.5,'speed',1);
            elseif sum(imcrop(green,[0 0 960 360]),'all')-sum(imcrop(green,[0 360 960 720]),'all')>10000
                moveup(drone,'distance',0.4,'speed',1);
            elseif sum(imcrop(green,[0 360 960 720]),'all')-sum(imcrop(green,[0 0 960 360]),'all')>10000
                movedown(drone,'distance',0.4,'speed',1);
            elseif sum(bw2,'all')<1000
                moveright(drone,'distance',0.5);
            end
        end
    end
```
- **구멍에 대한 중점 찾기**
```matlab
    %구멍에 대한 중점 찾기
    while 1
        %초록색에 대한 HSV값 설정 및 이진화
        frame=snapshot(cam);
        hsv = rgb2hsv(frame);
        h = hsv(:,:,1);
        s = hsv(:,:,2);
        v = hsv(:,:,3);
        green= (0.35<h)&(h<0.45)&(0.65<s)&(s<0.95);
        %첫 행을 1로 변환
        for i=1:960
            green(1,i)=1;
        end
        %마지막 행을 1로 변환
        for i=1:960
            green(720,i)=1;
        end
        %구멍을 채움
        bw2 = imfill(green,'holes');
        %구멍을 채우기 전과 후를 비교하여 값이 일정하면 0, 변했으면 1로 변환
        for x=1:720
            for y=1:960
                if green(x,y)==bw2(x,y)
                    bw2(x,y)=0;
                end
            end
        end
        %이미지에서 인식된 곳들의 중점과 보조축의 크기를 구함
        stats = regionprops('table',bw2, 'Centroid', 'MinorAxisLength');
        z=stats.MinorAxisLength;
        max=0;
        y=stats.Centroid; %중점 찾아주는 변수
        %보조축의 크기가 가장 큰 곳의 중점을 가져옴
        for i=1:size(stats)
            if z(i,1)>=max
                max=z(i,1);
                firstCenter(1,1)=round(y(i,1));
                firstCenter(1,2)=round(y(i,2));
            end
        end
        disp(firstCenter); %firstCenter 출력
        clearvars max %max변수 초기화
        %측정된 중점과 이상 중점을 비교하여 이동
        if firstCenter(1,1)-originCenter(level,1)>=40
            moveright(drone,'Distance',0.2,'speed',1);
            disp("right");
        elseif firstCenter(1,1)-originCenter(level,1)<=-40
            moveleft(drone,'Distance',0.2,'speed',1);
            disp("left");
        end
        if firstCenter(1,2)-originCenter(level,2)>=30
            movedown(drone,'Distance',0.3,'speed',1);
            disp("down");
        elseif firstCenter(1,2)-originCenter(level,2)<=-30
            moveup(drone,'Distance',0.2,'speed',1);
            disp("up");
        end
        %오차범위 내에 있으면 반복문 탈출
        if firstCenter(1,2)-originCenter(level,2)<30 && firstCenter(1,2)-originCenter(level,2)>-30 && firstCenter(1,1)-originCenter(level,1)<40 && firstCenter(1,1)-originCenter(level,1)>-40
            break;
        end
    end
    %1단계 및 2단계일 때 실행
    if level==1||level==2
        if level==1
            moveforward(drone,'Distance',2.6,'speed',1);
        elseif level==2
            moveforward(drone,'Distance',2.4,'speed',1);
        end
```
- **빨간점 인식**
```
        while 1
            %빨간색에 대한 HSV값 설정 및 이진화
            frame=snapshot(cam);
            hsv = rgb2hsv(frame);
            h = hsv(:,:,1);
            s = hsv(:,:,2);
            v = hsv(:,:,3);
            red= ((0.80<h)&(h<1)|(0<h)&(h<0.1))&(0.6<s)&(s<0.9)&(0.2<v)&(v<0.5);
            %빨간색의 픽셀이 400이 넘으면 90도 회전
            if sum(red,'all')>400
                if count==1
                    moveforward(drone,'distance',0.2);
                    count=0;
                end
                turn(drone,deg2rad(-90))
                break;
            else
                moveback(drone,'distance',0.2)
                count=1;
            end
        end
        %가운데 아래를 기준으로 하기위한 제어
        if level==1
            moveforward(drone,'Distance',0.8,'speed',1);
        elseif level==2
            moveforward(drone,'distance',1,'speed',1);
            moveright(drone,'Distance',0.2,'speed',1);
        end
        %3단계일 때 실행
    elseif level==3
        moveforward(drone,'Distance',2.1,'speed',1);
```
- **파란색 점 인식**
```
        while 1
            %파란색에 대한 HSV값 설정 및 이진화
            frame=snapshot(cam);
            hsv = rgb2hsv(frame);
            h = hsv(:,:,1);
            s = hsv(:,:,2);
            v = hsv(:,:,3);
            blue= (0.55<h)&(h<0.7)&(0.5<s)&(s<0.7)&(0.2<v)&(v<0.5);
            %파란색의 픽셀이 300이 넘으면 90도 회전
            if sum(blue,'all')>300
                if count==1
                    moveforward(drone,'distance',0.2);
                    count=0;
                end
                land(drone);
                break;
            else
                moveback(drone,'distance',0.2)
                count=1;
            end
        end
    end
end
```

# 미니드론 자율주행 경진대회 A리그 텔로토비팀!🚀

![드론관점주행](/readmeFile/dronemission.gif) 
<br>
## 1. 대회 진행 전략

1. 드론 이륙
- **이미지 전처리**
2. 텔로 드론의 영상을 받아와 [Snapshot](https://kr.mathworks.com/help/supportpkg/ryzeio/ref/snapshot.html) 함수를 이용하여 사진 영상을 가져옴
3. RGB 색 공간에서 HSV 색 공간으로 변경함
4. 초록색 링 통과를 위해 HSV 색 공간을 통하여 초록색을 찾고 링만 인식하기 위해 이진화를 실행함
5. 회전 점인 빨간색과 착지점인 파란색도 동일하게 이진화를 실행함

- **드론 제어**

6. 링이 왼쪽에 있는지 오른쪽에 있는지 판단하기 위해 영상을 반으로 나누어 각각 픽셀 수를 더하여 왼쪽이 많으면 왼쪽으로 ROLL 제어 이동하고, 오른쪽이 많으면 오른쪽으로 ROLL 제어 이동함
그리고 양쪽 값이 같다고 판단되면 PITCH 제어로 위로 올라가 링 구멍을 찾음
7. 이렇게 링을 찾은 뒤 링 구멍을 픽셀 좌표 찾는 방법을 사용하여 중심점을 찾음
8. 위에서 받은 Snapshot 영상의 중심점과 링 구멍 중심점의 차이를 계산 후 **X점의 차이가 ±49이면 좌우로 보정**하고, **Y점 차이가 ±30이면 상하로 보정**한 뒤 전진 함
9. 링 통과를 한 후 빨간색 원을 발견하게 되는데, 이 때 픽셀 수를 합하여 **500이 넘으면 좌회전**을 하고, **500이 넘지 못하면 후진**을 하여 다시 찾음
10. 마지막으로 파란색 원을 발견하게 되면 위와 같이 **500이 넘으면 착지**하고, 아니면 후진하여 다시 찾음

## 2. 알고리즘 설명

## 3. 소스코드 설명

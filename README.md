# linetracer1
로봇의 카메라를 이용해 트랙의 라인을 검출한다. 

gstreamer를 이용해 젯슨나노 보드에서 카메라 영상을 읽고, pc에 영상을 전송한다. 

관심영역을 지정하고, 평균 밝기를 지정해 밝기를 보정한 다음 영상을 이진화한다.

이진화 된 영상을 레이블링을 통해 영상속 검출되는 객체들의 개수와 무게중심값에 대한 정보를 확인 할 수 있다.

로봇의 초기 위치는 항상 라인 중심에 있고 영상의 중심에 있게 정의한다.

이후 검출되는 객체들 중 이전에 검출된 라인의 무게중심 좌표와 가장 가까운 객체를 라인으로 선정하게 한다.

이전 라인 무게중심과 현재 검출된 객체들의 무게중심의 오차가 가장 작은 객체를 최소값 알고리즘을 통해 찾아낸다.
![image](https://github.com/cubejun/linetracer1/assets/133946040/6f285624-731f-475b-9eda-8fec6ca2e0a4)

라인이 영상 밖으로 나가는 경우 최소값 알고리즘을 통해 검출되는 객체는 따라가던 라인이 아닌 잡음이나 다른 라인이 검출되는데, 이전 라인 무게중심과 현재 검출된 객체의 오차가 특정 값 이상이 되면 이전 라인 무게중심을 현재 라인의 무게중심으로 업데이트 되도록 만들었다.
![image](https://github.com/cubejun/linetracer1/assets/133946040/1c076114-8c4e-458a-9b5b-d72a35a9d1a9)

이렇게 검출된 라인의 무게중심을 영상의 중심좌표(로봇의 중심)에 빼는 것으로 라인이 로봇 정면에서 벗어난 정도(error)를 계산했다.

좌측속도명령 = 직진속도 - 게인\*error
우측속도명령 = 직진속도 + 게인\*error

s버튼을 누르면 모터가 가동되고, ctrl+c를 누르면 코드가 종료된다. 




https://github.com/cubejun/linetracer1/assets/133946040/b0b32f5d-dae9-4cdf-9de6-0b54e6ffc517



https://github.com/cubejun/linetracer1/assets/133946040/6c64ff6e-6a1e-4a2c-8059-51d68658111b



https://github.com/cubejun/linetracer1/assets/133946040/c8721bd9-0d74-469b-a4a0-68415f4d1618



https://github.com/cubejun/linetracer1/assets/133946040/d6ddb5da-f6fa-45df-9511-dbccad627f8b



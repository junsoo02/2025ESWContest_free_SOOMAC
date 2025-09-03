
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

import io
import os
import time
from google.cloud import vision

class YoloRealSenseNode(Node):
    """
    Combines YOLOv8 segmentation with RealSense depth sensing
    to detect objects and publish their 3D positions in a ROS 2 node.
    """
    def __init__(self):
        super().__init__('yolo_realsense_node')
        
        # Load the YOLOv8 segmentation model
        #욜로 모델과 세그멘테이션, ocr모델을 로드한다 
        self.model = YOLO("/home/pc/Downloads/drink_detect_seg2.pt")
        self.get_logger().info("YOLOv8 segmentation model loaded.")
        
        # Google Vision API 키 경로 설정
        os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = "/home/pc/Downloads/canocr-0ae4b75a8c3e.json"
        self.ocr_client = vision.ImageAnnotatorClient()

        # 리얼센스 파이프 라인 구축 
        self.pipeline = rs.pipeline()
        #데이터 스트림 활성화 
        self.config = rs.config()
        #깊이 데이터 사용,(640x480)해상도 설정 , 
        # 깊이 데이터 포맷 16비트 정수로 설정 "rs.format.z16" 16비트 정밀도로 깊이를 측정 
        #컬러 데이터의 포맷을 BGR 8비트로 설정 "rs.format.bgr8"
        # "30" 30fps로 데이터를 가져오게 함 -> 이거 높이면 높아지나? 해봐야할듯 
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        #앞에서 설정한 config라는 객체를 인자로 넘겨주며 파이프 라인을 시작 
        self.profile = self.pipeline.start(self.config)
        #리얼센스 시작 로그 
        self.get_logger().info("RealSense pipeline started.")


        #깊이와 컬러 스트림 정렬 
        # 리얼센스 카메라의 깊이센서와 컬러 센서는 물리적으로 떨어져 있음 -> 이를 align(정렬)해주는 역할을 함 
        #객체의 픽셀 위치를 기반으로 3D위치를 계산할 때 필수적인 과정 
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        #카메라 내부의 파라미터를 가져온다 깊이 데이터를 3D좌표로 변환할 때 사용 
        self.depth_intrinsics = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

        # Publishers 발행자 = 토픽을 생성 
        #drink_position -> position_publisher Int32 정수 타입의 메세지 
        self.position_publisher = self.create_publisher(Float32MultiArray, 'drink_position', 10)
        #scout_control -> control_publisher Int32 정수 타입의 메세지 
        self.control_publisher = self.create_publisher(Int32, 'scout_control', 10)

        # Subscribers 구독자 = 토픽을 
        self.drink_code_subscriber = self.create_subscription(
            Int32,
            'drink_code',
            self.drink_code_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.scout_realsense_subscriber = self.create_subscription(
            Int32,
            'scout_realsense',
            self.scout_realsense_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Class names for YOLO
        self.drink_names = ['Ddcan', 'Ycan', 'cass', 'pocari', 'samdasoo', 'zero_cola']
        self.model.names[5] = "zero_cola"

        # Keywords for zero_cola OCR
        self.zero_keywords = ["제로", "제로콜라", "zero", "zerosugar", "sugar", "zero sugar"]

        # State variables
        self.target_class_id = None
        self.is_transmitting = False
        self.is_drink_code_processed = False
        self.consecutive_detection_count = 0
        self.last_detection_time = time.time()
        self.coordinate_buffer = []

        # Timer for frame processing
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Node initialized with ROS 2 subscriptions and publishers.")

    def drink_code_callback(self, msg):
        """Callback to start one-shot coordinate transmission."""
        # 0~5 사이의 정수 값은 목표 음료를 설정합니다. (새로 추가된 로직)
        if 0 <= msg.data < len(self.drink_names):
            self.target_class_id = msg.data
            self.get_logger().info(f"Target object set to: {self.drink_names[self.target_class_id]}")
            self.is_transmitting = False
            self.is_drink_code_processed = False


        # 11이라는 특정 값은 일회성 좌표 전송을 시작합니다. (기존 로직)
        elif msg.data == 11 and not self.is_drink_code_processed:
            self.is_transmitting = True
            self.is_drink_code_processed = True
            self.get_logger().info("Received '11' on drink_code. Starting one-shot transmission.")
            self.last_detection_time = time.time()
            
        elif msg.data == 14:
            self.get_logger().info("Received '14' on drink_code. Resetting state and restarting detection.")
            self.target_class_id = None
            self.is_transmitting = False
            self.is_drink_code_processed = False
            self.consecutive_detection_count = 0
            self.coordinate_buffer = []

    def scout_realsense_callback(self, msg):
        """Callback to start continuous coordinate transmission."""
        if msg.data == 44:
            self.is_transmitting = True
            self.get_logger().info("Received '44' on scout_realsense. Starting continuous transmission.")
            self.last_detection_time = time.time()

    def stop_transmission_and_send_control(self, control_code):
        """Helper function to stop transmission and publish control signal."""
        self.is_transmitting = False
        self.coordinate_buffer = []
        control_msg = Int32()
        control_msg.data = control_code
        self.control_publisher.publish(control_msg)
        self.get_logger().info(f"Transmission stopped. Sent control code: {control_code}")

    def timer_callback(self):
        """Main loop for frame processing and logic."""
        
        try:
            #컬러와 깊이 센서를 사용할 수 있을때까지 기다림 
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()


            #댑스나 컬러 프레임이 하나라도 호출이 안될 경우 
            if not depth_frame or not color_frame:
                self.get_logger().warn("Skipping frame: No depth or color frame available.")
                return
            
            #객체 감지 시작 부분 컬러프레인 데이터를 넘파이 배열로 변환 
            color_image = np.asanyarray(color_frame.get_data())
            #self.model 미리 학습된 욜로8 객체 감지 모델 
            #verbode=false는 객체에 대한 자세한 로그 출력안하는 옵션
            results = self.model(color_image, verbose=False)

            detected_target = False


            #객체 감지 모델이 감지 결과를 반환 했는지 확인 
            #마스크 세그멘테이션 결과도 확인 
            #목표객체이 클래스 확인
            if results and results[0].masks is not None and self.target_class_id is not None:
                for mask_result, box_result in zip(results[0].masks, results[0].boxes):
                    class_id = int(box_result.cls.cpu().numpy()[0])
                    
                    #감지된 객체가 목표 객체인지 확인 맞다면 True로 변경 
                    if class_id == self.target_class_id:
                        detected_target = True

                        # 마스크 및 ROI(관심영역) 추출 
                        mask_np = mask_result.data[0].cpu().numpy().astype(np.uint8)
                        mask_resized = cv2.resize(mask_np, (color_image.shape[1], color_image.shape[0]))
                        
                        #폴리곤 포인트와 다각형 점들을 가져와서 가장 작은 사각형 (x,y,w,h)를 계싼 이는 바운딩 박스임
                        
                        polygon_points = np.array(mask_result.xy[0], dtype=np.int32)
                        x, y, w, h = cv2.boundingRect(polygon_points)
                        #roi 경계 상자좌표를 이용해 컬러 이미지에서 객체 부분만 자르고 roi에 저장함 
                        #-> 이는 객체 유효성과 ocr에 사용된다 
                        roi = color_image[y:y+h, x:x+w]
                        
                        # 객체 유효성 검증 부분 
                        #그림자 검증 
                        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                        v_channel = hsv_roi[:, :, 2]
                        dark_pixels = np.sum(v_channel < 40)
                        total_pixels = np.count_nonzero(mask_resized)
                        #전체 픽셀 수에서 어두운 픽셀 비율이 70%를 초과하면 객체는 잘못된 객체라고 판단 
                        if total_pixels > 0 and (dark_pixels / total_pixels) > 0.7:
                            continue
                        
                        #3D위치를 계산하기 위한 첫 단계 구체적으로 깊이 프레임에서 객체에 해당하는 부분만 남김
                        #나머지 배경부분의 깊이 정보는 모두 0으로 지움 
                        masked_depth = np.asanyarray(depth_frame.get_data()).copy()
                        #탐지된 객체의 영역에 해당하는 부분의 깊이정보만 남기고 나머지 배경ㅂ분의 깊이 정보는 모두
                        #0으로 지움 
                        masked_depth[mask_resized == 0] = 0
                        

                        #마스크가 적용된 깊이 배열에 유효한 깊이 값이 (0보다 큰값)이 잇는지 확인 없으면 다음단계로 
                        #넘어가지 않음 
                        if np.count_nonzero(masked_depth) > 0:
                            #배열에서 0보다 큰 모든 픽셀의 y와 x좌표를 각각 y_indices,x_indices 에 저장함
                            y_indices, x_indices = np.where(masked_depth > 0)
                            #center_x,center_y 객체에 속하는 모든 픽셀의 x좌표와 y좌표의 평균을 계산하여 
                            #((center_x, center_y))을 구함 
                            center_x = int(np.mean(x_indices))
                            center_y = int(np.mean(y_indices))
                            #이 중심점의 픽셀좌표를 사용하여 m단위로 가져온 뒤 1000을 곱하여 밀리미터 mm 단위로 변환 
                            distance_mm = depth_frame.get_distance(center_x, center_y) * 1000


                            #거리가 40mm에서 900mm인지 확인 
                            if 40 < distance_mm < 900:
                                #리얼센스 핵심 함수 카메라의 내부 파라미터 , 픽셀좌표 , 그리고 해당 픽셀의 거리
                                #distance_mm/1000을 입력받아 x,y,z 좌표로 변환 
                                point_3d = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [center_x, center_y], distance_mm / 1000)
                                #변환된 좌표에 100을 곱하여 cm로 변환 
                                x, y, z = [p * 100 for p in point_3d]
                               
                                #OCR 기반 클래스 재정의 
                                #객체 감지 모델이 예측한 클래스 ID에 해당하는 이름을 가져옴 
                                #객체가 ycan일 경우 ocr 로직을 실행함 
                                class_name = self.model.names[class_id]

                                if class_name.lower() == "ycan":
                                    # ROI를 jpg 형태로 압축하여 바이트 데이터로 만든다 
                                    #-> 구글 ocr클라이언트에 이미지를 보내기 위한 준비과정 
                                    is_success, buffer = cv2.imencode(".jpg", roi)
                                    if is_success:
                                        image_content = buffer.tobytes()
                                        image = vision.Image(content=image_content)
                                        try:
                                            #구글글라우드비전API를 호출하여 roi이미지에 포함된 텍스트를 인식
                                            response = self.ocr_client.text_detection(image=image)
                                            texts = response.text_annotations
                                            if texts:
                                                #인식된 테스트 전체를 가져와 소문자로 변환함 
                                                full_text = texts[0].description.lower()
                # zero_keywords 중 하나라도 포함되면 zero_cola로 변경
                                                if any(keyword in full_text for keyword in self.zero_keywords):
                                                    class_id = 5
                                                    class_name = "zero_cola"
                                                else:
                                                    class_name = "ycan"  # OCR 결과가 없을시에 원래 Ycan 유지
                                            else:
                                                class_name = "ycan" #ocr실패시 ycan 
                                        except Exception as e:
                                            self.get_logger().error(f"OCR failed: {e}")
                                            class_name = "ycan"

                                
                                #평균값 계산을 하는 부분 
                                #감지된 객체가 목표 객체인지 확인하고 신호를 받으면 위치전송을 시작함 
                                if class_id == self.target_class_id and self.is_transmitting:
                                    self.coordinate_buffer.append([z, -y, x])
                                    #목표 객체가 연속으로 감지될 때 마다 카운트를 1씩 증가시킴 
                                    self.consecutive_detection_count += 1
                                    #마지막으로 목표 객체를 감지한 시간을 기록함 
                                    self.last_detection_time = time.time()
                                    
                                    self.get_logger().info(
                                        f"Found target '{class_name}'. Consecutively detected: {self.consecutive_detection_count} frames."
                                    )
                                    
                                    # Check for 2-second continuous detection
                                    if self.consecutive_detection_count >= (2 / self.timer.timer_period_ns * 1e9): # 2초 동안 연속 인식
                                        #buffer에 쌓인 모든 3D좌표들을 평균낸다 
                                        avg_coords = np.mean(self.coordinate_buffer, axis=0)
                                        msg = Float32MultiArray()
                                        msg.data = list(avg_coords)

                                        #평균 좌표 메세지 5번 전송 
                                        for _ in range(5):
                                            self.position_publisher.publish(msg)
                                            self.get_logger().info(
                                                f"Published averaged coordinates for '{class_name}': X={avg_coords[0]:.2f}cm, Y={avg_coords[1]:.2f}cm, Z={avg_coords[2]:.2f}cm"
                                            )
        
                                        self.stop_transmission_and_send_control(21)
                                        
                                break # Stop after finding and processing the first valid target
            
            # If target not found in this frame, reset counter
            #연속 감지 초기화 및 타임아웃 처리 
            #타겟을 인식하지 못했을 경우 
            if not detected_target:
                self.consecutive_detection_count = 0
                self.coordinate_buffer = []

            #5초동안 찾지 못하면 22를 보냄
            if self.is_transmitting and (time.time() - self.last_detection_time) > 5:
                self.get_logger().warn("5-second timeout: Target object not found. Stopping transmission.")
                self.stop_transmission_and_send_control(22)

            #시각화 부분 
            #원본 컬러이미지에 그림을 그리기 위해 복사본을 만든다 
            result_image = color_image.copy()
            #감지 결과가 있고 마스크 정보가 있을 때만 시각화 작업을 시작
            if results and results[0].masks is not None:
                #마스크 그리기 객체의 윤곽을 이미지 위에 반투명한 녹색(colored_mask로 덧 씌움)
                for mask_result, box_result in zip(results[0].masks, results[0].boxes):
                    mask_np = mask_result.data[0].cpu().numpy().astype(np.uint8)
                    mask_resized = cv2.resize(mask_np, (color_image.shape[1], color_image.shape[0]))
                    #클래스 이름과 중심점 그림 
                    colored_mask = np.zeros_like(result_image, dtype=np.uint8)
                    colored_mask[:, :, 1] = mask_resized * 255
                    alpha = 0.5
                    result_image = cv2.addWeighted(result_image, 1, colored_mask, alpha, 0)
                    
                    class_id = int(box_result.cls.cpu().numpy()[0])
                    class_name = self.model.names[class_id]
                    
                    polygon_points = np.array(mask_result.xy[0], dtype=np.int32)
                    x, y, w, h = cv2.boundingRect(polygon_points)
                    center_x, center_y = x + w // 2, y + h // 2
                    
                    cv2.putText(result_image, f'{class_name}',
                                (center_x - 50, center_y - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.circle(result_image, (center_x, center_y), 5, (0, 0, 255), -1)


            #최종 결과 이미지를 inshow함 
            cv2.imshow("YOLOv8-seg + RealSense ROS2 Node", result_image)
            key = cv2.waitKey(1) #키보드 입력대기
            #종료 처리 
            if key & 0xFF == ord('q') or key == 27:
                self.get_logger().info('Shutting down node.')
                self.destroy_node()
                cv2.destroyAllWindows()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloRealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    finally:
        node.pipeline.stop()
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

> 과거 구현의 실험 기록입니다. 현재 단계형 FSM과 실행 방법은 `static_obstacle_avoidance.md`를 따릅니다. 별도 도로 경계 검사는 삭제되었습니다.

# S4 Stage 5B — K-City 전체 원본 지도 검증

검사일: 2026-11-09 (Asia/Seoul). 기준 브랜치 `s4-finish`, HEAD `7845817`.

**최종 선택은 2번: 추가 지도 파일·속성 자료가 필요하다.** 현재 자료로 승인 가능한
과거 도로 경계 검사 후보는 0개다. 원본에 관측된 경계·시설선과 긴 차로 구조는 있으므로,
그 자료를 대상으로 추가 검증을 할 근거는 있다. 추가 자료의 실제 보유·제공 가능성은
확인되지 않았다. 확보 후에도 안전 검증과 외부 승인이 필요하다.

Stage 5A의 합성 폐루프 성공은 실제 K-City 주행 가능 판정과 별개다. 이번에는
실제 지도 회피 경로, 주행용 Polygon/cell, 승인 SHA256, MGeo 허가 속성을 생성하지 않았다.

## 1. 원본 ZIP과 검사 범위

원본: `/workspace/local_data/map_original/SEC01_첨단주행시험로.zip`.
SHA256: `38fb36fc654d6417e83167d2e2b7266eb38524d6d9167def920f9492bdeee463`.
파일 160개, 비압축 16,246,922바이트를 모두 CRC/hash 검사했다. 압축파일은 읽기 전용으로
열었고 추출하지 않았다. 기존 `kcity_hdmap` 40개 파일은 ZIP의 UTM52N 타원체고 자료와
모두 바이트 동일하다. 기존 UTM52N 추출본에서 빠진 레이어는 없다.

| 묶음 | 파일 수 | 확인 내용 |
|---|---:|---|
| HDMap_UTM52N_타원체고 | 40 | SHP/SHX/DBF/PRJ 10세트 |
| HDMap_UTMK_타원체고 | 40 | 같은 10개 레이어의 좌표 변환본 |
| HDMap_UTMK_정표고 | 40 | 같은 10개 레이어의 높이 변환본 |
| ControlPoint/2018·2019 | 39 | PointZ 7세트, 총 127레코드 및 인덱스/메타데이터 |
| NGII Excel | 1 | 도화데이터, 점군데이터, 코드표, 도화데이터_v1.0 전체 셀 |

ZIP에는 PDF/HWP 납품 사양서, Lanelet2/OpenDRIVE, LAS/LAZ 원점군, 별도 도로 표면
레이어가 없다. Excel의 점군 메타데이터는 원점군 자체가 아니다. 기존 Stage 3의 공식
2019 NGII 매뉴얼은 ZIP 밖의 참고 자료이며, 이 납품본에 대한 정의 적용성·완전성
증명으로 취급하지 않았다.

## 2. 전체 레이어와 참조

아래 개수·필드 집합·ID는 좌표계 3종에서 같다. 전체 필드의 타입/폭/결측·코드 분포,
기하/좌표 범위와 참조 감사는 [전체 감사 JSON](/workspace/logs/s4_stage5b_20261109/full_audit/audit.json),
[레이어 CSV](/workspace/logs/s4_stage5b_20261109/full_audit/layer_inventory.csv)에 있다.

| 레이어 | 개수 | 기하 | 핵심 근거/한계 |
|---|---:|---|---|
| A1_NODE | 712 | PointZ | 연결 노드 |
| A2_LINK | 809 | PolyLineZ | LinkType 6 일반 463개, 1 교차로 346개; 인접/시종점/Section 참조 |
| A3_DRIVEWAYSECTION | 6 | PolygonZ | 모두 Kind 1/RoadType 1; 국부 구간, 전체 도로면 아님 |
| B1_SAFETYSIGN | 198 | PointZ | 표지 종류·LinkID·PostID·참조 차로 |
| B2_SURFACELINEMARK | 1,574 | PolyLineZ | Type/Kind, 좌우 링크; 관측 차선·규제선 |
| B3_SURFACEMARK | 301 | PolygonZ | 노면 표시 면, 일반 도로 표면과 다름 |
| C1_TRAFFICLIGHT | 81 | PointZ | 신호등·링크·지주 |
| C3_VEHICLEPROTECTIONSAFETY | 690 | PolyLineZ | 시설/연석, 안·밖/차로 대응이 없는 선 |
| C4_SPEEDBUMP | 6 | PolygonZ | 방지턱 면, 전체 주행 가능 영역이 아님 |
| C6_POSTPOINT | 92 | PointZ | 지주 |

모든 비어 있지 않은 검사 대상 FK는 대상 ID에 연결된다. A2의 A3 SectionID는 10개에만
있고 799개는 비어 있다. 이는 A3를 전체 도로면으로 사용할 근거가 아니다. 연결 노드와
형상 끝점에 약 1mm 차이가 있는 링크는 380/477/480이며, 임의 snap을 하지 않았다.
전체 809개에서 인접한 일반 차로 114쌍, 정확히 이어지는 체인 58개를 분석했다.
교차로 통과나 분기를 임의로 이어 붙이지 않았다.

## 3. 좌표계·높이·측량 품질

UTM52N은 PRJ의 EPSG:32652와 MGeo `UTM52N`이 일치한다. 원점은
`(302459.942, 4122635.537, 28.991)`이다. UTMK는 제공 WKT를 그대로 사용했다.
WKT 제목은 ITRF2000이지만 DATUM은 WGS84로 기재돼 있어 임의로 EPSG:5179를 부여하지 않았다.

같은 정점 배열의 재투영 XY 차이는 최대 약 0.000101m다. C3 8개 피처는 정점 수가
달라 정점별 동일성으로 처리하지 않고 양방향 정점→원본 선분으로 별도 대조했다.
그 XY 차이도 최대 약 0.000101m였다. 추가 도로 경계/도로면 레이어는 아니다.
이 수치는 절대 측량 정확도 또는 연속 Hausdorff 오차 인증이 아니다.

타원체고 두 자료는 Z가 사실상 같고, 정표고 자료와의 관측 Z 차이는 약
22.5669~22.6170m다. 공식 지오이드/수직 기준·정확도 보고서는 없으므로 상수 높이
보정이나 MGeo 수직 기준 확정을 하지 않는다.

새로 확인한 품질 문제:

- Excel `도화데이터!CF4`는 C3 705개, 실제 세 좌표계 모두 690개다. 추출 누락이 아니라
  원본 payload와 메타데이터의 차이다. 어떤 15개가 누락됐는지, 메타데이터가 오래됐는지는 알 수 없다.
- UTMK 정표고의 A2/A3/B2/B3/C3/C4 SHP 6개는 헤더 길이와 실제 길이가 다르다.
  SHX/DBF가 참조하는 레코드는 진단 판독으로 대조했고 미참조 후행 바이트는 각각
  88/336/136/72/152/144바이트다. 엄격 검사 실패를 남겼으며 원본을 잘라 고치지 않았다.
- `ControlPoint/2018/GCP_Laser_Measure.prj`가 없다. 가까운 좌표라는 이유로 CRS를 부여하지 않았다.
- GCP Ground 46개 중 30개는 SHP Z=0인데 DBF Z는 약 28m다. 예: A15는 0 대 28.538m.
  CP-415의 DBF와 SHP XY 차이는 약 0.2284m다. 어느 표현이 측량 기준인지 미확인이다.
- 2018 MMP laser/vector의 같은 ID 20개는 XY 최대 약 0.1072m, Z 약 0.0752m 차이다.
  이를 MORAI 좌표 등록 정확도 또는 측량 오차 인증으로 사용하지 않는다.

세부 값: [좌표계 비교](/workspace/logs/s4_stage5b_20261109/full_audit/coordinate_variants.csv),
[측량점별 대조](/workspace/logs/s4_stage5b_20261109/full_audit/control_point_coordinate_checks.csv).

## 4. 실제 경계·주행 가능 영역·통행 규칙의 존재 여부

**관측된 선은 있다. 승인할 수 있는 완전한 도로면/제외 영역은 확인되지 않았다.**
B2에는 길가장자리 구역선 Kind 505가 570개, 중앙선 501이 355개, 안전지대선 531이
121개, 진로변경제한선 506이 52개 있다. C3에는 Type 4 연석 628개, 가드레일 16개,
방호벽 4개, 벽 8개 등이 있다. 차로 경계와 물리 시설의 추가 조사 근거로 쓸 수 있다.

C3는 모두 LowHigh=2, isCentral=0, Ref_ID는 전부 비어 있다. 관측 폐곡선 7개를
도로 외곽/교통섬으로 임의 분류하거나 안쪽을 채우지 않았다. isCentral=0이나 A3 Kind 7
부재도 중앙분리대·자율주행 금지 영역이 없다는 증거가 아니다. A3 6개는 약 56×65m의
국부 범위, 총 약 876.65m²이며 전체 ordinary-road surface 및 holes를 제공하지 않는다.

B2 Type 212/Kind 503 등은 차로 횡단 검토의 단서다. 그러나 실제 구간·방향·시점별
진입 및 복귀 허가 자료는 없다. Type 214 두 건은 자동 해석하지 않는다.
MGeo 352개에는 `can_move_*_lane` 필드가 없으며 기존 S4 엄격 정책의 허용 인접 방향은 0개다.
표시선 코드에서 허가 boolean을 생성하지 않았다.

B1에는 SubType 211 세 건, 212 두 건 등 진입/진행 규제 표지도 있다. 적용 방향과 구간은
별도 검토 대상이며 도로 전체 금지 면으로 변환하지 않았다. 모든 후보에 B1/C1/C4의 원본
LinkID 참조를 기록했다. 상위 후보 1·2는 연결 표지/신호/방지턱 참조가 없고, 후보 3에는
B119BS010114(SubType 218)와 B119BS010125(199)가 있다. 참조 부재도 규제 없음의 증명은 아니다.

## 5. Stage 3 재검토

MGeo 352개=일반 254+lazy 98, 공통 A2 ID 253개, B2 1,574개, 공통 ID 중 양측 표시선
참조 링크 131개를 원본과 다시 확인했다. 일반 MGeo `L1`은 대응 SHP가 없다.
공통 253개는 링크 유형/시종점/인접 ID가 일치하며 표본 XY 차이는 최대 0.1731m,
Z 차이는 0.02682m다. 실제 장면 등록 오차의 보증은 아니다.

기존 엄격 도구를 다시 실행하면 전체 법선 지지 통과는 `398` 한 개, 부분 지지 링크는
75개다. 과거 94개 1차 검사, 이 whole-link 검사, 이번 58개 체인은 단위와 기준이
다르므로 통과 수로 합치지 않는다. `075↔096`의 공통 B219BS010377과 약 21.7m 길이,
`624↔625`의 교차로 유형/공통 B2 부재도 재확인했다. 새 전체 원본에는 더 긴 SHP 체인이
있지만 그 존재가 기존 두 쌍의 결손을 보완하지는 않는다.

## 6. 전체 후보와 상위 3개

58개 중 A=0, B=54, C=4다. 이는 지도 근거 분류이며 Lattice 생성/차량 추종 성공률이 아니다.
전체 [후보 CSV](/workspace/logs/s4_stage5b_20261109/full_audit/candidates.csv)에 점수와 거부/미해결 사유가 있다.
점수는 길이·법선 표본 지지·완만한 점선 구간·MGeo ID 지원을 정렬하는 용도다.
진입·복귀 허가나 도로 표면 승인을 대체하지 않는다.

분석 가정은 3km/h, 장애물 앞면 45m/길이 4m, 20m 진입·복귀 전이다.
기존 rollout 6.433m와 차체/말단 여유, 시작 뒤 차체 1.109m를 포함해 약 85.441m의
관측 지원을 비교했다. 1m 간격 및 원래 중심선 정점에서 표시선 순서·인접 중심선 포함을 검사했다.
완만함 `|κ|≤1/250m`는 후보 정렬 가정이며 운영 게이트가 아니다. 표본 지지는 연속 안전 인증이 아니다.

| 후보 | 원래 차선 체인 길이 | 3표시선 최대 표본 지지 | 완만한 점선 표본 구간 | 현재 MGeo | 분류 |
|---|---:|---:|---:|---|---|
| 1: 306/307 시작 7쌍 | 538.54m | 164.00m | 30.00m | 14/14 ID | B |
| 2: 205↔207 | 701.30m | 701.00m | 185.14m | 0/2 ID | B |
| 3: 277/278 시작 5쌍 | 836.39m | 218.60m | 86.00m | 0/10 ID | B |

후보 1은 현재 MGeo와 일치하지만 위 정렬 가정에서 약 85m의 완만한 연속 구간이 없다.
다른 곡률에서 반드시 불가능하다는 뜻은 아니다. 도로면·허가 검증도 남아 있다.
후보 2는 긴 검토 구간과 B2 공통 `B219BS010787`, 바깥 표시선 `B219BS010786/010788`이
확인된다. 그러나 이들은 두 차로의 표시선이며 실제 도로 외곽이라는 뜻은 아니다.
현재 MGeo/장면 대응, 도로면/holes, 통행 허가가 필요하다.
후보 3도 같은 누락이 있고 정렬 조건의 여유는 86.00−85.44≈0.56m로 작다.

## 7. 재현 가능한 지도 그림

[전체 지도](/workspace/logs/s4_stage5b_20261109/full_audit/whole_map_evidence.png),
[후보 1](/workspace/logs/s4_stage5b_20261109/full_audit/candidate_1/map_detail.png),
[후보 2](/workspace/logs/s4_stage5b_20261109/full_audit/candidate_2/map_detail.png),
[후보 3](/workspace/logs/s4_stage5b_20261109/full_audit/candidate_3/map_detail.png).

각 폴더에 전체 체인 `map_evidence.png/svg`, 확대 `map_detail.png/svg`, 표본 CSV,
evidence.json 및 request_scope.json이 있다. MGeo/SHP/B2/C3/기타 원본 면과 결손을 구분한다.
색칠한 진입/유지/복귀 구간은 관측 중심선 위 길이 예시다. 실제 회피 궤적이나 주행 영역이 아니다.
확대 그림의 축 회전은 가독성을 위한 좌표 표시이며 원본 지도 변환/수정이 아니다.

## 8. 부족한 자료와 다음 해결 방법

**우선 후보 2의 약 185m 구간만 대상으로 다음 자료의 제공 가능성을 확인한다.**
원래 링크 station 약 26.86~212.00m, 예시 전체 지원은 26.86~112.30m다.
링크 A219BS010205/A219BS010207 및 위 B2 ID를 함께 지정한다.
request_scope.json의 경위도 bbox는 중심선 참조 위치이며 도로 전체 면적/폭을 뜻하지 않는다.

1. 차로 좌우 경계와 실제 외곽선, 연석/섬/중앙분리대·진입 금지·도로면 holes를 포함한
   검토·측량된 면 또는 경계 위상 자료. 도로 표면 Polygon, 내용이 충족되는 Lanelet2/OpenDRIVE,
   MORAI/MGeo 차로 경계 자료 등은 요청 형식의 예이며 실제 존재를 가정하지 않는다.
2. 해당 구간 양방향 진입·복귀의 허용 여부, 유효 station/방향/시간·차선 통제 조건과
   적용 스키마/코드 정의서. 점선만으로 자동 승인하지 않는다.
3. 해당 SHP ID가 포함된 실제 MORAI 장면과 MGeo 대응본, 장면 revision, 좌표·차량 기준점
   등록 및 검증 결과. 현재 MGeo 누락은 장면 자체의 부재를 뜻하지 않는다.
4. C3 705/690 차이에 대한 정정 메타데이터 또는 누락/삭제 ID·이력, 정표고 SHP 6개의
   정상 재납품본, `GCP_Laser_Measure.prj`, Ground DBF/SHP 중 권위 좌표와 높이·정확도 설명,
   적용 수직 기준/지오이드 모델. 임의 복구/높이 보정은 하지 않는다.

이 자료가 제공되지 않으면 같은 ZIP을 반복 분석해도 승인 조건이 채워지지 않는다.
그때는 별도로 측량·통제 가능한 시험 환경에서 동일 알고리즘을 검증하는 편이 효율적이다.
자료가 확보되더라도 현재 21.19m ROI, 미승인 관측/제동 corridor, Stage 5A 실제 계산시간의
freshness 문제는 별도로 해결해야 한다. 이번 단계에서 센서 범위나 게이트는 바꾸지 않았다.

## 9. 도구·테스트·보존

추가 파일은 `tools/kcity_full_map_audit.py`,
`test/test_kcity_full_map_audit.py`, 이 문서다. stdlib/numpy/pyproj/matplotlib만 사용하며
새 라이브러리 설치, Docker/ROS 환경 변경, 운영 패키지 빌드·노드 재시작은 하지 않았다.

관련 기존 테스트 13개와 신규 15개, 총 **28개 통과**.
[pytest 결과](/workspace/logs/s4_stage5b_20261109/targeted_tests.txt),
[JUnit XML](/workspace/logs/s4_stage5b_20261109/targeted_tests.xml).
기존 801개 전체 회귀 테스트를 이번 단계에서 재실행한 것은 아니다.
운영 소스와 설정을 포함한 기존 추적 파일 140개를 바이트/hash 대조해 모두 동일함을 확인했다.
원본 ZIP·HDMap·MGeo 44개 파일도 해시가 동일하다. 새 판독기는 기존 UTM52N 판독기와
10개 레이어 전 레코드의 속성·기하가 정확히 일치한다.

`enable_drive=false`, 빈 과거 경계 파일/승인 설정, 기존 Dijkstra·PP·PID·Lattice와
정상 글로벌 패스 기준선을 유지했다. 실제 MORAI 구동 및 commit/push는 하지 않았다.

## 10. 오프라인 재현

```bash
cd /workspace
MPLCONFIGDIR=/tmp/edgenode_stage5b_mpl PYTHONDONTWRITEBYTECODE=1 \
  python3 tools/kcity_full_map_audit.py \
  --zip /workspace/local_data/map_original/SEC01_첨단주행시험로.zip \
  --hdmap /workspace/local_data/kcity_hdmap \
  --mgeo /workspace/local_data/kcity_mgeo \
  --out /tmp/kcity_stage5b_diagnostic

source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 -m pytest -q -p no:cacheprovider \
  ros2_ws/src/edgenode_planning/test/test_kcity_full_map_audit.py \
  ros2_ws/src/edgenode_planning/test/test_hdmap_audit.py
```

도구는 원본 디렉터리 안으로의 출력을 거부한다. 모든 결과는 분석 artifact이며
과거 도로 경계 검사 loader에 넣을 수 있는 스키마를 만들지 않는다.


# draft
##  __perception.cc:__
 在入口函数中加载适配器的配置文件，注册所有的传感器和对象数据类以及节点。初始化时启动`dag_streaming`，后续工作交由`dag_streaming`处理。

* ## __dag_streaming__
    `dag_streaming`继承自线程类，以线程的方式启动，构造该类时，创建`DAGStreamingMonitor`类。`dag_streaming`的初始化函数完成以下4个任务：
    * 加载配置文件并解析
    * 初始化`EventManager`类<br/>
      EventManager主要负责边Edge的初始化，通过该类统一管理各个topic消息的订阅与发布。EvenManager类包含两个主要的成员变量，分别保存<事件id，消息队列>的event_queue_map_，以及保存<事件id，事件信息>的event_meta_map_(用于调试，打印信息)。一个事件的成分包含id和name。一个完整的Edge总体保存了事件信息(id，name)，入度节点(from_node)，出度节点(to_node)。
    * 初始化数据共享类`SharedDataManager`<br/>
    从配置文件读取需要处理的数据类型，可以通过该配置文件选择指定传感器的数据。
    * 初始化各个子节点<br/>
    `DAGStreaming::InitSubnodes`函数完成了一个有向图的初始化，先配置处理数据的节点，再配置各个边，最后执行初始化。

    初始化完成以后，调用`Schedule`函数启动各个子节点，节点以线程方式启动。[参考链接](https://github.com/YannZyl/Apollo-Note/blob/master/docs/perception/perception_software_arch.md)

* ## __Fusion_subnode__
    * Init功能
    1. 注册概率融合和异步融合两种融合算法。
      2. 设置底盘回调，获取速度

* ## __sensor extrinsic__

    __navigation mode__

  - __usage of radar extrinsic__

    radar_process_subnode.cc: 
    
       ```c++
        RadarProcessSubnode::InitInternal(){
        ...
        std::string radar_extrinstic_path = FLAGS_radar_extrinsic_file;
          AINFO << "radar extrinsic path: " << radar_extrinstic_path;
          Eigen::Affine3d radar_extrinsic;
          if (!LoadExtrinsic(radar_extrinstic_path, &radar_extrinsic)) {
            AERROR << "Failed to load extrinsic.";
            return false;
          }
          radar_extrinsic_ = radar_extrinsic.matrix();
          AINFO << "get radar extrinsic succ. pose: \n" << radar_extrinsic_;
     
          std::string short_camera_extrinsic_path = FLAGS_short_camera_extrinsic_file;
          AINFO << "short camera extrinsic path: " << short_camera_extrinsic_path;
          Eigen::Affine3d short_camera_extrinsic;
          if (!LoadExtrinsic(short_camera_extrinsic_path, &short_camera_extrinsic)) {
            AERROR << "Failed to load extrinsic.";
            return false;
          }
          short_camera_extrinsic_ = short_camera_extrinsic.matrix();
          AINFO << "get short camera  extrinsic succ. pose: \n"
                << short_camera_extrinsic_;
          inited_ = true;

        ...

        }
       ```

       ```c++
        void RadarProcessSubnode::OnRadar(const ContiRadar &radar_obs) {
        ...
        
        if (!FLAGS_use_navigation_mode) {
        *radar2world_pose =
            *velodyne2world_pose * short_camera_extrinsic_ * radar_extrinsic_;
        ADEBUG << "get radar trans pose succ. pose: \n" << *radar2world_pose;

          } else {
            CalibrationConfigManager *config_manager =
                Singleton<CalibrationConfigManager>::get();
            CameraCalibrationPtr calibrator = config_manager->get_camera_calibration();
            // Eigen::Matrix4d camera_to_car = calibrator->get_camera_extrinsics();
            *radar2car_pose = radar_extrinsic_;
            ADEBUG << "get radar trans pose succ. pose: \n" << *radar2car_pose;
          }

        ...

        // 4. Call RadarDetector::detect.
          PERF_BLOCK_START();
          if (!FLAGS_use_navigation_mode) {
            options.radar2world_pose = &(*radar2world_pose);
          } else {
            options.radar2world_pose = &(*radar2car_pose);
          }
          std::shared_ptr<SensorObjects> radar_objects(new SensorObjects);
          radar_objects->timestamp = timestamp;
          radar_objects->sensor_type = SensorType::RADAR;
          radar_objects->sensor2world_pose = *radar2world_pose;
          bool result = radar_detector_->Detect(radar_obs_proto, map_polygons, options, 
                                            &radar_objects->objects);
        }
       ```
       modest_radar_detector.cc : 
       ```c++
        bool ModestRadarDetector::Detect(...){

        ...

        Eigen::Matrix4d radar_pose;
          if (options.radar2world_pose == nullptr) {
            AERROR << "radar2world_pose is nullptr.";
            return false;
          } else {
            radar_pose = *(options.radar2world_pose);
          }
        }

        ...

        SensorObjects radar_objects;
        object_builder_.Build(raw_obstacles, radar_pose, main_velocity,
                                &radar_objects);
        ...
       ```
     object_builder.cc
       ```c++
        bool ObjectBuilder::Build(...){
        ...

        Eigen::Matrix<double, 4, 1> location_r;
        Eigen::Matrix<double, 4, 1> location_w;
        location_r << raw_obstacles.contiobs(i).longitude_dist(),
        raw_obstacles.contiobs(i).lateral_dist(), 0.0, 1.0;
        location_w = radar_pose * location_r;

        ...

        Eigen::Matrix<double, 3, 1> velocity_r;
        Eigen::Matrix<double, 3, 1> velocity_w;
        velocity_r << raw_obstacles.contiobs(i).longitude_vel(),
            raw_obstacles.contiobs(i).lateral_vel(), 0.0;
        velocity_w = radar_pose.topLeftCorner(3, 3) * velocity_r;

        ...
        
        object_ptr->position_uncertainty =
            radar_pose.topLeftCorner(3, 3) * dist_rms * dist_rms.transpose() *
            radar_pose.topLeftCorner(3, 3).transpose();
        object_ptr->velocity_uncertainty =
            radar_pose.topLeftCorner(3, 3) * vel_rms * vel_rms.transpose() *
            radar_pose.topLeftCorner(3, 3).transpose();
        double local_theta =
            raw_obstacles.contiobs(i).oritation_angle() / 180.0 * M_PI;
        Eigen::Vector3f direction =
            Eigen::Vector3f(cos(local_theta), sin(local_theta), 0);
        direction = radar_pose.topLeftCorner(3, 3).cast<float>() * direction;
        object_ptr->direction = direction.cast<double>();

        ...
        }
       ```
      
  - __usage of camera extrinsic__
     
    通过`camera_process_subnode.cc`，创建处理camera数据的子节点，在创建该节点时，首先执行`bool CameraProcessSubnode::InitInternal()`初始化函数，该函数调用`InitCalibration()`, 在`InitCalibration()`内通过获得`CalibrationConfigManager`的单例对象进而获得相机的内参和外参。
    
    ```c++
    bool CameraProcessSubnode::InitInternal() {
      // Subnode config in DAG streaming
      std::unordered_map<std::string, std::string> fields;
      SubnodeHelper::ParseReserveField(reserve_, &fields);

      if (fields.count("device_id")) device_id_ = fields["device_id"];
      if (fields.count("pb_obj") && stoi(fields["pb_obj"])) pb_obj_ = true;
      if (fields.count("pb_ln_msk") && stoi(fields["pb_ln_msk"])) pb_ln_msk_ = true;

      // Shared Data
      cam_obj_data_ = static_cast<CameraObjectData *>(
          shared_data_manager_->GetSharedData("CameraObjectData"));
      cam_shared_data_ = static_cast<CameraSharedData *>(
          shared_data_manager_->GetSharedData("CameraSharedData"));

      InitCalibration();

      InitModules();

      AdapterManager::AddImageFrontCallback(&CameraProcessSubnode::ImgCallback,
                                            this);
      if (pb_obj_) {
        AdapterManager::AddChassisCallback(&CameraProcessSubnode::ChassisCallback,
                                           this);
      }
      return true;
    }
    bool CameraProcessSubnode::InitCalibration() {
      auto ccm = Singleton<CalibrationConfigManager>::get();
      CameraCalibrationPtr calibrator = ccm->get_camera_calibration();
      calibrator->get_image_height_width(&image_height_, &image_width_);
      camera_to_car_ = calibrator->get_camera_extrinsics();
      intrinsics_ = calibrator->get_camera_intrinsic();
      return true;
    }
    ```
      查看`calibration_config_manager.cc`的构造函数可知，其加载的是长焦相机的内参和外参。
      ```c++
        CalibrationConfigManager::CalibrationConfigManager()
          : camera_calibration_(new CameraCalibration()) {
          work_root_ = FLAGS_work_root;
          camera_extrinsic_path_ = FLAGS_front_camera_extrinsics_file;
          camera_intrinsic_path_ = FLAGS_front_camera_intrinsics_file;
          init();
        }  
      ```
      `init()`函数调用`init_internal()`，在`init_internal()`内初始化`CameraCalibration`类的实例指针`CameraCalibrationPtr`.
      ```c++
        bool CalibrationConfigManager::init() {
          MutexLock lock(&mutex_);
          return init_internal();
        }
        bool CalibrationConfigManager::init_internal() {
          if (inited_) {
            return true;
          }
          if (!camera_calibration_->init(camera_intrinsic_path_,
                                         camera_extrinsic_path_)) {
            AERROR << "init intrinsics failure: " << camera_intrinsic_path_ << " "
                   << camera_extrinsic_path_;
            return false;
          }
          AINFO << "finish to load Calibration Configs.";
          inited_ = true;
          return inited_;
        }
      ```
    
    在`CameraCalibration`类的初始化函数中最终把相机外参、分辨率以及内参信息加载到内存并存储到变量中。

    ```c++
      bool CameraCalibration::init(const std::string &intrinsic_path,
                               const std::string &extrinsic_path) {
        if (!camera_coefficient_.init("", extrinsic_path, intrinsic_path)) {
          AERROR << "init camera coefficient failed";
          return false;
        }
        camera_intrinsic_ = camera_coefficient_.camera_intrinsic;
        image_height_ = camera_coefficient_.image_height;
        image_width_ = camera_coefficient_.image_width;
        *_camera2car_pose = camera_coefficient_.camera_extrinsic;
        *_car2camera_pose = _camera2car_pose->inverse();

        init_camera_model();
        calculate_homographic();
        AINFO << "Successfully loading intrinsic and extrinsic";
        return true;
      }
    ```

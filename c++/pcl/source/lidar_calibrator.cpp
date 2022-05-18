//绕八字，得到最接近一圈的一组数据
//求ldiar里程计
//手眼标定求主lidar位姿
//点云配准求其他lidar位姿


//车辆位姿差值去畸变
//车辆位姿给出初始估计
//pcl::registration::CorrespondenceEstimation 评估对应关系,
//CorrespondenceEstimation是确定目标和查询点集(或特征)之间的对应关系的基类，
//输入为目标和源点云，输出为点对，即输出两组点云之间对应点集合。
//pcl::registration::TransformationEstimationPointToPlane 计算对应关系点间的坐标变换
//迭代匹配

PointMatchResult PclPointToPlane(const std::vector<Vec3d>& ref_points,
  const std::vector<Vec3d>& src_points,
  const AffineTransformation& init_transform,
  double dis_thres) {
  Timer time_elapsed;
  //点云初始化
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud =
    util::ToPclPointCloud(ref_points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud =
    util::ToPclPointCloud(src_points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned(
    new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointNormal>::Ptr ref_cloud_normals(
    new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*ref_cloud, *ref_cloud_normals);

  //法向估计
  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> normal_estimation;
  normal_estimation.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(
    new pcl::search::KdTree<pcl::PointNormal>));
  normal_estimation.setKSearch(20);
  normal_estimation.setInputCloud(ref_cloud_normals);
  normal_estimation.setViewPoint(0, 0, 10);
  normal_estimation.compute(*ref_cloud_normals);

  //对应关系
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
  int iteration = 0;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  // Convergence criteria

  //收敛准则初始化
  pcl::registration::DefaultConvergenceCriteria<float> convergence_criteria(
    iteration, transform, *correspondences);
  convergence_criteria.setMaximumIterations(kMaxIteration);
  convergence_criteria.setMaximumIterationsSimilarTransforms(5);
  convergence_criteria.setRelativeMSE(1e-5);

  // Correspondence estimation
  boost::shared_ptr<
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>>
    cor_est_closest_point(
      new pcl::registration::CorrespondenceEstimation<pcl::PointXYZ,
      pcl::PointXYZ>());
  cor_est_closest_point->setInputTarget(ref_cloud);

  // Transformation estimation
  pcl::registration::TransformationEstimationPointToPlane<pcl::PointXYZ,
    pcl::PointNormal>
    transformation_estimation;
  Eigen::Matrix4f initial_estimate = init_transform.mat().cast<float>();
  pcl::transformPointCloud(*src_cloud, *source_aligned, initial_estimate);
  Eigen::Matrix4f final_transform = initial_estimate;

  const float correspondence_threshold = dis_thres;
  do {
    cor_est_closest_point->setInputSource(source_aligned);
    cor_est_closest_point->determineCorrespondences(*correspondences,
      correspondence_threshold);

    transform.setIdentity();
    transformation_estimation.estimateRigidTransformation(
      *source_aligned, *ref_cloud_normals, *correspondences, transform);
    final_transform = transform * final_transform;

    pcl::transformPointCloud(*src_cloud, *source_aligned, final_transform);

    iteration++;
  } while (
    !convergence_criteria.hasConverged() /*&& iteration <= kMaxIteration*/ &&
    time_elapsed.endWithSecond() < kMaxIterationTime);

  // estimate matching points number
  int match_count = 0;
  double mse = 0;
  Vec3d avg_diff;
  std::vector<Vec3d> point_diffs;

  for (const auto& elem : *correspondences) {
    if (elem.index_match == -1) continue;
    const auto& point_target = ref_cloud->points[elem.index_match];
    const auto& point_source = source_aligned->points[elem.index_query];
    const Vec3d vec_diff(point_target.x - point_source.x,
      point_target.y - point_source.y,
      point_target.z - point_source.z);
    const double dist = vec_diff.norm();
    match_count++;
    mse += dist;
    avg_diff += vec_diff;
    point_diffs.push_back(vec_diff);
  }

  PointMatchResult result;
  if (match_count <= 1) {
    result.success = false;
    return result;
  }

  CHECK_GE(match_count, 1);
  avg_diff /= match_count;

  Mat3d diff_cov;
  diff_cov.setZero();
  for (const Vec3d& pt_diff : point_diffs) {
    diff_cov += (pt_diff - avg_diff) * (pt_diff - avg_diff).transpose();
  }
  diff_cov *= 1.0 / (match_count - 1);

  result.num_matched_points = match_count;
  result.matched_ratio = 1.0 * match_count / src_points.size();
  result.mse =
    match_count > 0 ? mse / match_count : std::numeric_limits<double>::max();
  result.transform = Mat4fToAffineTransformation(final_transform);
  result.success =
    time_elapsed.endWithSecond() < kMaxIterationTime - 1 ? true : false;
  result.diff_cov_diag = { diff_cov(0, 0), diff_cov(1, 1), diff_cov(2, 2) };
  LOG(INFO) << "xxxxxxxxxxxxx diff_cov_diag: "
    << result.diff_cov_diag.transpose() << " mse:" << result.mse
    << " norm:" << result.diff_cov_diag.norm()
    << " iter_num:" << iteration << "  elapsed:" << time_elapsed.end();

  // final
  return result;
}


//帧间匹配求lidar位姿
void GetLidarOdometry(const std::vector<PointCloudXYZI::Ptr>& point_clouds,
  const std::vector<double>& origin_timestamps,
  const std::vector<AffineTransformation>& body_odometry,
  const AffineTransformation& lidar_extrinsics,
  const int thread_num,
  std::vector<AffineTransformation>* lidar_odometry,
  std::set<int>* bad_match_indices, double* average_mse) {
  QCHECK_GT(origin_timestamps.size(), 1);
  QCHECK(bad_match_indices->empty());
  int size = origin_timestamps.size();
  QCHECK_EQ(body_odometry.size(), size - 1);
  lidar_odometry->reserve(size - 1);
  lidar_odometry->resize(size - 1);
  std::vector<bool> match_valid_record(size - 1, true);
  std::vector<double> match_mse(size - 1, 0);

  ThreadPool thread_pool(thread_num);
  ParallelFor(0, size - 1, &thread_pool, [&](int i) {
    LOG_EVERY_N(INFO, 20) << "Getting lidar odometry, index = " << i;
    std::vector<Vec3d> ref_points, src_points;
    for (const auto& p : point_clouds[i]->points) {
      ref_points.push_back(Vec3d{ p.x, p.y, p.z });
    }
    for (const auto& p : point_clouds[i + 1]->points) {
      src_points.push_back(Vec3d{ p.x, p.y, p.z });
    }
    AffineTransformation init_transform =
      lidar_extrinsics.Inverse() * body_odometry[i] * lidar_extrinsics;
    const PointMatchResult match = mapping::util::PclPointToPlane(
      ref_points, src_points, init_transform, 1.0);

    LOG_IF(WARNING, !match.success)
      << "Lidar odometry match failed!! Index is " << i;
    if (!match.success) match_valid_record[i] = false;

    lidar_odometry->at(i) = match.transform;
    match_mse[i] = match.mse;
    });
  *average_mse = 0.0;
  for (int i = 0; i < size - 1; ++i) {
    if (!match_valid_record[i])
      bad_match_indices->insert(i);
    else
      *average_mse += match_mse[i];
  }
  *average_mse /= (match_mse.size() - bad_match_indices->size());
}

//手眼标定
void HandEyeCalibration(const std::vector<AffineTransformation>& body_odometry,
  const std::vector<AffineTransformation>& lidar_odometry,
  const std::set<int>& bad_match_indices,
  AffineTransformation* lidar_extrinsics,
  double* angle_precision,
  double* translation_precision) {
  QCHECK_EQ(body_odometry.size(), lidar_odometry.size());
  std::vector<AffineTransformation> valid_body_odometry = body_odometry;
  std::vector<AffineTransformation> valid_lidar_odometry = lidar_odometry;
  for (auto it = bad_match_indices.crbegin(); it != bad_match_indices.crend();
    ++it) {
    valid_body_odometry.erase(valid_body_odometry.begin() + *it);
    valid_lidar_odometry.erase(valid_lidar_odometry.begin() + *it);
  }
  int size = valid_body_odometry.size();
  /**
   * We use b for Body, and l for Lidar.
   * when calibrating rotation, i.e. R :
   * R*tl = Rb * t - t + tb; R*φl = φb. So we define matrix L to augment φl
   * and tl, and define matrix B to augment φb and (Rb * t - t + tb). when
   * calibrating translation, i.e. t : (Rb - I) * t = R*tl - tb. So we define
   * matrix Q to augment (Rb - I), and define matrix V to augment (R*tl - tb);
   */
  Eigen::Matrix3d R = lidar_extrinsics->mat().block(0, 0, 3, 3);
  Eigen::Vector3d t = lidar_extrinsics->mat().block(0, 3, 3, 1);
  Eigen::MatrixXd L(3, 2 * size);
  Eigen::MatrixXd B(3, 2 * size);
  std::vector<Eigen::Matrix3d> Rbs(size);
  std::vector<Eigen::Vector3d> tbs(size);
  std::vector<Eigen::Matrix3d> Rls(size);
  std::vector<Eigen::Vector3d> tls(size);
  for (int i = 0; i < size; ++i) {
    Eigen::Matrix3d Rb, Rl;
    Eigen::Vector3d tb, tl;
    Rb = valid_body_odometry[i].mat().block(0, 0, 3, 3);
    tb = valid_body_odometry[i].mat().block(0, 3, 3, 1);
    Rl = valid_lidar_odometry[i].mat().block(0, 0, 3, 3);
    tl = valid_lidar_odometry[i].mat().block(0, 3, 3, 1);
    Eigen::AngleAxisd phi_b(Rb);
    Eigen::AngleAxisd phi_l(Rl);
    // augment B and L
    B.col(2 * i) = phi_b.angle() * phi_b.axis();
    B.col(2 * i + 1) = Rb * t + tb - t;
    L.col(2 * i) = phi_l.angle() * phi_l.axis();
    L.col(2 * i + 1) = tl;
    // save
    Rbs[i] = Rb;
    tbs[i] = tb;
    Rls[i] = Rl;
    tls[i] = tl;
  }
  // calculate R : 传统定姿问题
  Eigen::Matrix3d lbt = L * B.transpose();
  Eigen::Matrix3d MtM = lbt.transpose() * lbt;
  R = MtM.sqrt().inverse() * lbt.transpose();
  // calculate t
  Eigen::MatrixXd Q(3 * size, 3);
  Eigen::MatrixXd V(3 * size, 1);
  for (int i = 0; i < size; ++i) {
    Q.block(i * 3, 0, 3, 3) = Rbs[i] - Eigen::Matrix3d::Identity();
    V.block(i * 3, 0, 3, 1) = R * tls[i] - tbs[i];
  }
  Eigen::Matrix3d qtq = Q.transpose() * Q;
  Eigen::Vector3d qtv = Q.transpose() * V;
  double z = t[2];
  t = qtq.inverse() * qtv;
  t[2] = z;
  // update extrinsics
  Eigen::Matrix4d final_transform = Eigen::Matrix4d::Identity();
  final_transform.block(0, 0, 3, 3) = R;
  final_transform.block(0, 3, 3, 1) = t;
  *lidar_extrinsics = AffineTransformation::FromMat4d(final_transform);
  // calculate precision
  AffineTransformation ex = *lidar_extrinsics, ex_inv = ex.Inverse();
  *angle_precision = 0;
  *translation_precision = 0;
  for (int i = 0; i < size; ++i) {
    AffineTransformation residual_transform = ex_inv * valid_body_odometry[i] *
      ex *
      valid_lidar_odometry[i].Inverse();
    double residual_angle = residual_transform.GetRotationAngle();
    double residual_translation = residual_transform.GetTranslation().norm();
    *angle_precision += residual_angle * residual_angle;
    *translation_precision += residual_translation * residual_translation;
  }
  *angle_precision = std::sqrt(*angle_precision / size);
  *translation_precision = std::sqrt(*translation_precision / size);
}


//匹配多帧点云，计算均值
void CalibrationBetweenLidars(
  const std::vector<PointCloudXYZI::Ptr>& target_clouds,
  const std::vector<PointCloudXYZI::Ptr>& source_clouds, bool point_to_plane,
  const int thread_num, AffineTransformation* lidar_lidar_extrinsics,
  MatchPrecision* calib_precision) {
  QCHECK_EQ(target_clouds.size(), source_clouds.size());

  int size = target_clouds.size();
  std::vector<bool> match_valid_record(size, true);
  std::vector<AffineTransformation> match_results(size);
  std::vector<double> match_mse(size);
  // async scan match
  ThreadPool thread_pool(thread_num);
  ParallelFor(0, size, &thread_pool, [&](int i) {
    LOG_EVERY_N(INFO, 40) << "Matching between lidars, index = " << i;
    std::vector<Vec3d> ref_points, src_points;
    for (const auto& p : target_clouds[i]->points) {
      ref_points.push_back(Vec3d{ p.x, p.y, p.z });
    }
    for (const auto& p : source_clouds[i]->points) {
      src_points.push_back(Vec3d{ p.x, p.y, p.z });
    }
    AffineTransformation init_transform = *lidar_lidar_extrinsics;
    const PointMatchResult match =
      point_to_plane ? mapping::util::PclPointToPlane(ref_points, src_points,
        init_transform, 1.0)
      : mapping::util::PclPointToPoint(ref_points, src_points,
        init_transform, 1.0);

    LOG_IF(WARNING, !match.success)
      << "Lidar-Lidar match failed!! Index is " << i;
    if (!match.success) match_valid_record[i] = false;
    match_results[i] = match.transform;
    match_mse[i] = match.mse;
    });
  // Normalization to get output extrinsics
  std::vector<Mat3d> valid_match_R_results;
  std::vector<Vec3d> valid_match_t_results;
  Vec3d accumulate_t = Vec3d::Zero();
  double average_mse = 0.0;
  for (int i = 0; i < size; ++i) {
    if (match_valid_record[i]) {
      valid_match_R_results.push_back(match_results[i].mat().block(0, 0, 3, 3));
      valid_match_t_results.push_back(match_results[i].GetTranslation());
      accumulate_t += match_results[i].GetTranslation();
      average_mse += match_mse[i];
    }
  }

  Mat3d avr_R = ComputeMeanValueForRoatationMatrices(valid_match_R_results);
  Vec3d avr_t = accumulate_t / valid_match_t_results.size();
  Eigen::Matrix4d final_transform = Eigen::Matrix4d::Identity();
  final_transform.block(0, 0, 3, 3) = avr_R;
  final_transform.block(0, 3, 3, 1) = avr_t;
  *lidar_lidar_extrinsics = AffineTransformation::FromMat4d(final_transform);
  // calculate precision
  int valid_size = valid_match_R_results.size();
  calib_precision->avr_mse = average_mse / valid_size;
  calib_precision->angle_std = 0.0;
  calib_precision->translation_std = 0.0;

  for (int i = 0; i < valid_size; ++i) {
    double residual_angle =
      Eigen::AngleAxisd(avr_R.transpose() * valid_match_R_results[i]).angle();
    double residual_translation = (avr_t - valid_match_t_results[i]).norm();
    calib_precision->angle_std += residual_angle * residual_angle;
    calib_precision->translation_std +=
      residual_translation * residual_translation;
  }
  calib_precision->angle_std =
    std::sqrt(calib_precision->angle_std / valid_size);
  calib_precision->translation_std =
    std::sqrt(calib_precision->translation_std / valid_size);
}

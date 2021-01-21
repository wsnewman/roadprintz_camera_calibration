%ul_corner = [427.2,221.5]
%ll_corner = [435.8,470.2]
%dist_ul_ll = dist(ul_corner,ll_corner)
clear all
image_gridpoints = load("poster_virtual_gridpoints.csv");
lidar_gridpoints = load("image_from_pointcloud_fwd_gridpoints.csv");
lidar_gridpoints_rvrs = load("image_from_pointcloud_rvrs_gridpoints.csv");

[rms_err] = match_rms_err(lidar_gridpoints, image_gridpoints)

figure(1)
clf
plot(image_gridpoints(:,1),image_gridpoints(:,2),'r*')
hold on
plot(lidar_gridpoints(:,1),lidar_gridpoints(:,2),'b*')
plot(lidar_gridpoints_rvrs(:,1),lidar_gridpoints_rvrs(:,2),'gs')

hold off

lidar_pts_centroid = mean(lidar_gridpoints)
lidar_pts_rvrs_centroid = mean(lidar_gridpoints_rvrs)
diff_lidar_pts_centroid_fwd_rvrs = lidar_pts_centroid-lidar_pts_rvrs_centroid

image_pts_centroid = mean(image_gridpoints)
centroid_diff_fwd_lidar_vs_image = lidar_pts_centroid-image_pts_centroid
centroid_diff_rvrs_lidar_vs_image = lidar_pts_rvrs_centroid-image_pts_centroid

image_pts_centered = image_gridpoints-image_pts_centroid;
lidar_pts_centered = lidar_gridpoints-lidar_pts_centroid;
lidar_pts_rvrs_centered = lidar_gridpoints_rvrs-lidar_pts_rvrs_centroid;


figure(2)
clf
plot(image_pts_centered(:,1),image_pts_centered(:,2),'r*')
hold on
plot(lidar_pts_centered(:,1),lidar_pts_centered(:,2),'b*')
plot(lidar_pts_rvrs_centered(:,1),lidar_pts_rvrs_centered(:,2),'gs')
hold off

%lidar_pts_centered-lidar_pts_rvrs_centered
%pause


rms_err_min = 1000;
theta_best = -10;
for theta= -0.01:0.0001:0.01
  
  Rotz=[cos(theta), -sin(theta); sin(theta), cos(theta)];
  rot_pts = (Rotz*lidar_pts_centered')';
   [rms_err] = match_rms_err(rot_pts, image_pts_centered);
   if (rms_err<rms_err_min)
     rms_err_min=rms_err;
     theta_best = theta;
   end
end

display("end of loop")
theta_best
rms_err_min
  Rotz=[cos(theta_best), -sin(theta_best); sin(theta_best), cos(theta_best)];
  rot_pts = (Rotz*lidar_pts_centered')';
   [rms_err] = match_rms_err(rot_pts, image_pts_centered)

rms_err_min = 1000;
theta_best_rvrs = -10;
for theta= -0.01:0.0001:0.01
  
  Rotz=[cos(theta), -sin(theta); sin(theta), cos(theta)];
  rot_pts = (Rotz*lidar_pts_rvrs_centered')';
   [rms_err] = match_rms_err(rot_pts, image_pts_centered);
   if (rms_err<rms_err_min)
     rms_err_min=rms_err;
     theta_best_rvrs = theta;
   end
end   
theta_best_rvrs

  Rotz=[cos(theta_best_rvrs), -sin(theta_best_rvrs); sin(theta_best_rvrs), cos(theta_best_rvrs)];

  rot_pts = (Rotz*lidar_pts_rvrs_centered')';
   [rms_err_rvrs] = match_rms_err(rot_pts, image_pts_centered)   
dtheta=theta_best_rvrs-theta_best
  
%keypts_offset-template_pts_offset
    figure(3)
      clf
      plot(rot_pts(:,1),rot_pts(:,2),'r*')
      hold on
      plot(image_pts_centered(:,1),image_pts_centered(:,2),'b*')
      hold off     

rot_offset_pts = rot_pts+image_pts_centroid;
    figure(4)
      clf
      plot(rot_offset_pts(:,1),rot_offset_pts(:,2),'r*')
            hold on

            plot(rot_offset_pts(:,1),rot_offset_pts(:,2),'r')
      plot(rot_offset_pts(1,1),rot_offset_pts(1,2),'rd', "markersize", 20)

      plot(image_gridpoints(:,1),image_gridpoints(:,2),'b*')
            plot(image_gridpoints(1,1),image_gridpoints(1,2),'bh', "markersize", 20)
      hold off    
      
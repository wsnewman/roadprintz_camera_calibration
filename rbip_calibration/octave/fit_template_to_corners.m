%ul_corner = [427.2,221.5]
%ll_corner = [435.8,470.2]
%dist_ul_ll = dist(ul_corner,ll_corner)
clear all
keypts = load("image_from_pointcloud_fwd_keypoints.csv")
KPIX_RESCALED = 590/4
%dist_metric = dist_ul_ll/KPIX_RESCALED
template_pts = [];
%key pts for a portait-mode poster, starting from ur corner and rastering down to lr corner
%18x8 poster intersections
for row=1:8
   u = -(row-1)*KPIX_RESCALED*0.1007;
  for col=1:18
    v= (col-1)*KPIX_RESCALED*0.1007;
    template_pts=[template_pts;[u,v]];
  end
end
template_centroid = mean(template_pts)
keypts_centroid = mean(keypts)
template_pts_offset = template_pts-template_centroid;
keypts_offset = keypts-keypts_centroid;
figure(1)
clf
plot(template_pts_offset(:,1),template_pts_offset(:,2),'r*')
hold on
plot(keypts_offset(:,1),keypts_offset(:,2),'b*')
hold off

rms_err_min = 1000;
theta_best = -10;
for theta= -0.1:0.001:0.1
  theta
  Rotz=[cos(theta), -sin(theta); sin(theta), cos(theta)];
  rot_pts = (Rotz*template_pts_offset')';
   [rms_err] = match_rms_err(rot_pts, keypts_offset);
   if (rms_err<rms_err_min)
     rms_err_min=rms_err
     theta_best = theta
   end
end

display("end of loop")
theta_best
rms_err_min
  Rotz=[cos(theta_best), -sin(theta_best); sin(theta_best), cos(theta_best)]
  rot_pts = (Rotz*template_pts_offset')';
   [rms_err] = match_rms_err(rot_pts, keypts_offset)
   
  
%keypts_offset-template_pts_offset
    figure(2)
      clf
      plot(rot_pts(:,1),rot_pts(:,2),'r*')
      hold on
      plot(keypts_offset(:,1),keypts_offset(:,2),'b*')
      hold off     

rot_offset_pts = rot_pts+keypts_centroid
    figure(3)
      clf
      plot(rot_offset_pts(:,1),rot_offset_pts(:,2),'r*')
            hold on

            plot(rot_offset_pts(:,1),rot_offset_pts(:,2),'r')
      plot(rot_offset_pts(1,1),rot_offset_pts(1,2),'rd', "markersize", 20)

      plot(keypts(:,1),keypts(:,2),'b*')
            plot(keypts(1,1),keypts(1,2),'bh', "markersize", 20)
      hold off    
      
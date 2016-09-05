% AFRL GOTCHA ground truth velocity calibration
% The given ground truth velocity contains its Magnitude and Angle in the
% xy-plane, thus the z velocity is missing
% 
% Base on GPS, we estimate the z velocity. Since simple diff gives us very
% coarse discrete values, we smooth the diff to give the estimation.
% 
% Mar 15, 2016

vx_s = (-sin(2*pi-truth_SPIE.trgHdg_degTrue/180*pi).*truth_SPIE.trgSpeed_mps);
vy_s = (cos(2*pi-truth_SPIE.trgHdg_degTrue/180*pi).*truth_SPIE.trgSpeed_mps);
vz_s = diff(smooth(truth_SPIE.trgPos_PCS(3,:)));
vz_s = [vz_s;vz_s(end)];

diffx=diff(truth_SPIE.trgPos_PCS(1,:));diffx=[diffx diffx(end)];
diffy=diff(truth_SPIE.trgPos_PCS(2,:));diffy=[diffy diffy(end)];
diffz=diff(truth_SPIE.trgPos_PCS(3,:));diffz=[diffz diffz(end)];

ExtraDataLenFront=17; 
for i=1:70   % sec by sec
    % target velocity
    vec1=[-sin(2*pi-truth_SPIE.trgHdg_degTrue(i+ExtraDataLenFront)/180*pi) ...
        cos(2*pi-truth_SPIE.trgHdg_degTrue(i+ExtraDataLenFront)/180*pi) 0];
    vec3=[-sin(2*pi-truth_SPIE.trgHdg_degTrue(i+ExtraDataLenFront)/180*pi)*truth_SPIE.trgSpeed_mps(i+ExtraDataLenFront) ...
        cos(2*pi-truth_SPIE.trgHdg_degTrue(i+ExtraDataLenFront)/180*pi)*truth_SPIE.trgSpeed_mps(i+ExtraDataLenFront) vz_s(i+ExtraDataLenFront)];
    vec4=[diffx(i+ExtraDataLenFront) diffy(i+ExtraDataLenFront) diffz(i+ExtraDataLenFront)];
    % slant range vector
    vec2=truth_SPIE.trgPos_PCS(:,i+ExtraDataLenFront)-hdrs.APCpos(:,floor(aux_SPIE.PRF*(i)));
    CosTheta(i) = dot(vec1,vec2)/(norm(vec1)*norm(vec2));
    CosTheta2(i) = dot(vec3,vec2)/(norm(vec3)*norm(vec2));
    CosTheta3(i) = dot(vec4,vec2)/(norm(vec4)*norm(vec2));
    radial_vel_xy(i)=CosTheta(i) *truth_SPIE.trgSpeed_mps(i+ExtraDataLenFront);
    radial_vel_z(i)=CosTheta2(i) *truth_SPIE.trgSpeed_mps(i+ExtraDataLenFront);
    radial_vel_gps(i)=CosTheta3(i) *truth_SPIE.trgSpeed_mps(i+ExtraDataLenFront);
end

figure;plot(radial_vel_xy,'b');hold on;
plot(radial_vel_gps,'r');
plot(radial_vel_z,'g');
legend('xy','gps','xy + gpsZ')


vx_select = -sin(2*pi-truth_SPIE.trgHdg_degTrue/180*pi).*truth_SPIE.trgSpeed_mps;
vy_select = cos(2*pi-truth_SPIE.trgHdg_degTrue/180*pi).*truth_SPIE.trgSpeed_mps;
vz_select = vz_s;


for i=1:70
    azimuth_vel_z(i)= sqrt(vx_select(i+ExtraDataLenFront).^2 + vy_select(i+ExtraDataLenFront).^2 + vz_select(i+ExtraDataLenFront).^2 - radial_vel_z(i).^2);
        
end





% Means and standard deviations:
clc;
ind_start1 = 250;
ind_start2 = 250;
ind_start3 = 250;

% Linear
avg_linear_1 = mean(d1_mod_acc(ind_start1:end)); % m/s²
std_linear_1 =  std(d1_mod_acc(ind_start1:end)); % m/s²

avg_linear_2 = mean(d2_mod_acc(ind_start2:end)); % m/s²
std_linear_2 =  std(d2_mod_acc(ind_start2:end)); % m/s²

avg_linear_3 = mean(d3_mod_acc(ind_start3:end)); % m/s²
std_linear_3 =  std(d3_mod_acc(ind_start3:end)); % m/s²

disp(["  Drone 1 linear avg +- std: "+ avg_linear_1 + " +- " + std_linear_1]);
disp(["  Drone 2 linear avg +- std: "+ avg_linear_2 + " +- " + std_linear_2]);
disp(["  Drone 3 linear avg +- std: "+ avg_linear_3 + " +- " + std_linear_3]);


% Yaw

B = 1/30*ones(30,1);
d1_acc.a = filter(B, 1, d1_acc.a);
d2_acc.a = filter(B, 1, d2_acc.a);
d3_acc.a = filter(B, 1, d3_acc.a);

avg_yaw_1 = mean(d1_acc.a(ind_start1:end, 3)); % rad/s²
std_yaw_1 =  std(d1_acc.a(ind_start1:end, 3)); % rad/s²

avg_yaw_2 = mean(d2_acc.a(ind_start2:end, 3)); % rad/s²
std_yaw_2 =  std(d2_acc.a(ind_start2:end, 3)); % rad/s²

avg_yaw_3 = mean(d3_acc.a(ind_start3:end, 3)); % rad/s²
std_yaw_3 =  std(d3_acc.a(ind_start3:end, 3)); % rad/s²

disp(["  Drone 1 yaw avg +- std: "+ avg_yaw_1 + " +- " + std_yaw_1]);
disp(["  Drone 2 yaw avg +- std: "+ avg_yaw_2 + " +- " + std_yaw_2]);
disp(["  Drone 3 yaw avg +- std: "+ avg_yaw_3 + " +- " + std_yaw_3]);

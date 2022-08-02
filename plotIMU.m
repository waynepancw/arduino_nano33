ax = out.Acceleration(:,1);
ay = out.Acceleration(:,2);
az = out.Acceleration(:,3);

gx = out.AngularRate(:,1);
gy = out.AngularRate(:,2);
gz = out.AngularRate(:,3);

mx = out.MagneticField(:,1);
my = out.MagneticField(:,2);
mz = out.MagneticField(:,3);

figure
subplot(3,1,1)
plot(ax)
hold on
plot(ay)
hold on
plot(az)

subplot(3,1,2)
plot(gx)
hold on
plot(gy)
hold on
plot(gz)

subplot(3,1,3)
plot(mx)
hold on
plot(my)
hold on
plot(mz)

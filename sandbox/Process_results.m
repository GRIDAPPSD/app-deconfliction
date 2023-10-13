clc
clear all


file_dir = 'outputs\';
% case_name = 'No_block_test_2';
% duration = 1;
case_name = 'block_test_2';
duration = 15;

RMin= 201.5914
RMax= 821.2414
% DMax = 65.1925;
DMax = 64.5;
DMin = 67.0327;


measurements = readtable(strcat(file_dir, case_name, '_meas.csv'));
preferences = readtable(strcat(file_dir, case_name, '_app_preferences.csv'));
data = read_json(strcat(file_dir, case_name, '_conflict_store.json'));



time_steps =  linspace(0,24, 97);
time_steps = time_steps(1:end-1)';

ret_batt1 = [];
ret_batt2 = [];
measurements(1,:) = [];
for idx = 2:height(measurements)
    
    if measurements.PBatt1(idx) ~= measurements.PBatt1(idx-1)
        ret_batt1 = [ret_batt1;  (measurements.PBatt1(idx)- measurements.PBatt1(idx-1))/250];
        ret_batt2 = [ret_batt2;  (measurements.PBatt2(idx)- measurements.PBatt2(idx-1))/250];
    end
end
    
S_batt1 = std(ret_batt1);
S_batt2 = std(ret_batt2);

volume_batt1 = length(ret_batt1)
volume_batt2 = length(ret_batt2)

volatality_batt1 = sqrt(length(volume_batt1))* S_batt1
volatality_batt2 = sqrt(length(volume_batt2))* S_batt2

R_method = mean(measurements.socBatt1*500) + mean(measurements.socBatt2)*500
D_method = sum(max(measurements.Psub, 0))*duration/(60*1000)
D_success = (DMin - D_method)/(DMin -DMax)
R_DM_sucess = (R_method - RMin)/(RMax -RMin)

case_name = 'block_test_2';
measurements = readtable(strcat(file_dir, case_name, '_meas.csv'));




%%% Plotting %%%
figure
subplot(2,1,1);
yyaxis left
plot(measurements.Time, measurements.PBatt1, 'Marker', '*', 'MarkerSize',3, 'LineStyle', '-', 'LineWidth',1.5)
hold on 
plot(preferences.Time, preferences.Res_Batt1, 'Color', "r", 'LineStyle', ':', 'LineWidth', 1.6)
plot(preferences.Time, preferences.Decarb_Batt1, 'Color', "g",'LineStyle', ':', 'LineWidth', 1.6)
ylabel('Battery #1 Output (kW)','FontSize',12, 'FontName','Times New Roman','fontweight','bold') 
ylim([-275,275])
yyaxis right
plot(measurements.Time, measurements.socBatt1, 'Marker', 'o', 'MarkerSize',3, 'LineStyle', '--', 'LineWidth',1.25)
ylabel('Battery #1 SOC','FontSize',12,'FontName','Times New Roman','fontweight','bold')
xlabel('Time of Day (Hr)','FontSize',12,'FontName','Times New Roman','fontweight','bold')
legend('Dispatch','Res-Preferred','Decarb-Preferred', 'SOC')
grid on
xlim([0,24])

subplot(2,1,2);
yyaxis left
plot(measurements.Time, measurements.PBatt2, 'Marker', '*', 'MarkerSize',3, 'LineStyle', '-', 'LineWidth',1.5)
ylabel('Battery #2 Output (kW)','FontSize',12,'FontName','Times New Roman','fontweight','bold') 
hold on 
plot(preferences.Time, preferences.Res_Batt2, 'Color', "r", 'LineStyle', ':', 'LineWidth', 1.6)
plot(preferences.Time, preferences.Decarb_Batt2, 'Color', "g",'LineStyle', ':', 'LineWidth', 1.6)
ylim([-275,275])
yyaxis right
plot(measurements.Time, measurements.socBatt2, 'Marker', 'o', 'MarkerSize',3, 'LineStyle', '--', 'LineWidth',1.25)
ylabel('Battery #2 SOC','FontSize',12,'FontName','Times New Roman','fontweight','bold')
xlabel('Time of Day (Hr)','FontSize',12,'FontName','Times New Roman','fontweight','bold') 
legend('Dispatch','Res-Preferred','Decarb-Preferred', 'SOC')
grid on

function val = read_json(fname)

fid = fopen(fname); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
val = jsondecode(str);

end
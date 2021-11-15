%% Create some random plot
time = linspace(0,10,100);
v_sin = sin(time);
v_cos = cos(time);

hfig = figure(1);
plot(time,v_sin)
hold on;
plot(time,v_cos,'r')
hold off;
legend('Robot a','Robot b')
xlabel('time [$s$]')
ylabel('velocity [$\frac{m}{s}$]')

% Create tikz file
matlab2tikz('figurehandle', hfig, 'example_tikz_figure.tikz', 'height', '\fheight', 'width', '\fwidth', 'parseStrings', false);

% Create mlf2pdf figure
mlf2pdf(hfig,'example_matlabfrag_figure')
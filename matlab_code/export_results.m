function export_results(folderName)

% Save all open figures

if nargin == 0
    folderName = fullfile(pwd,'Figures');
end

% Create the folder if it doesn't exist
if ~exist(folderName,'dir')
    mkdir(folderName);
end

% Get all open figures
figs = findall(groot,'Type','figure');

% Save each figure
for k = 1:length(figs)

    filename = fullfile(folderName,...
        sprintf('Figure_%d.png',k));

    exportgraphics(figs(k),filename,'Resolution',300);

end

fprintf('\nAll figures saved successfully!\n');
fprintf('Location:\n%s\n',folderName);

end
%% KEMS UAV - Single Master PDF Publisher
% Runs and publishes the entire simulation codebase and all figures
% into a single, comprehensive PDF document ready for submission.

clear;
clc;
close all;

disp('===========================================================');
disp('   KEMS UAV: Publishing Full Codebase to Single PDF');
disp('===========================================================');

opts.format = 'pdf';
opts.outputDir = fullfile(pwd, 'Published_Submission_PDF');
opts.showCode = true;
opts.evalCode = true;
opts.catchError = true;
opts.createThumbnail = false;
opts.maxHeight = 600;
opts.maxWidth = 800;

if ~exist(opts.outputDir, 'dir')
    mkdir(opts.outputDir);
end

disp('Publishing complete project report...');
pdf_output = publish('publish_complete_project_report.m', opts);

disp('===========================================================');
disp(['[SUCCESS] PDF Created: ', pdf_output]);
disp('===========================================================');

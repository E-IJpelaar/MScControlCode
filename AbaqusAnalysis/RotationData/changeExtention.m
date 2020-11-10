function done = changeExtention()

file = dir('*.rpt');            % list all files in path with .rpt extension
current_extention = '.rpt';     % curent extension .rpt
new_extention = '.txt';         % change to .txt extension
for ii = 1:length(file)         % copy the files so original .rpt is maintained
new_name = regexprep(file(ii).name,current_extention,new_extention);
[~, ~, ~]  = copyfile(file(ii).name, new_name, 'f');
end
done  =1;
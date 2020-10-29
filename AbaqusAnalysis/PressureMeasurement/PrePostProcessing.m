clear all;close all;clc;tic


%% Change input (.rpt) to text (.txt) so Matlab can read it
file = dir('*.rpt');
current_extension = '.rpt';
new_extension = '.txt';
for ii = 1:length(file)
new_name = regexprep(file(ii).name,current_extension,new_extension);
[status, message, ~]  = copyfile(file(ii).name, new_name, 'f');
end

%% Get Node numbers of the Top Plate
filename = 'TopNodes60kMesh.txt' ;
fid = fopen(filename,'rt') ;
S = textscan(fid,'%s','Delimiter','\n');
S = S{1} ;
%%Get the line number 
idxNode = strfind(S, '--------------------');  % search the text for this expression
idx1 = find(not(cellfun('isempty', idxNode))); % find index in which previous expression is found
idxNodeEnd = strfind(S, 'Minimum');            % search the text for this expression
idx2 = find(not(cellfun('isempty', idxNodeEnd)));   % find index in which previous expression is found
% pick  nodes 
nodes = S(idx1+1:idx2-3);                      % between those 2 expression node numbers are given
Nodes = zeros(length(nodes),5);                % pre-allocate
for ii = 1:length(nodes)
    Nodes(ii,:) = str2num(nodes{ii});          % write data in matrix
end
activeNodes = Nodes(:,1);                      % make a collum of active nodes

%% Get orignal and deformed x,y,z from top plate
fname = '90kPa.txt' ;                 % load data from all nodes
fid = fopen(fname,'rt') ;
K = textscan(fid,'%s','Delimiter','\n');
K = K{1} ;
%%Get the line number of mises 
idxNode = strfind(K, '--------------------'); % find this exppression
idx1 = find(not(cellfun('isempty', idxNode)));% find index of this expression
% pick  nodes 
allnodes = K(idx1(1)+1:idx1(2)-4);            % all nodes can be found between this expression
AllNodes = zeros(length(allnodes),7);
for ii = 1:length(allnodes)
    AllNodes(ii,:) = str2num(allnodes{ii}(20:end)); % write all data in matrix 
end

%% Create One matrix with active node xyz
% [#node u_x u_y u_z d_x d_y d_z]
data = zeros(length(activeNodes),7);
for ii = 1:length(activeNodes)
    noi = activeNodes(ii);
    [row,~] = find(AllNodes(:,1) == noi);

    data(ii,:) = AllNodes(row,:);
end
toc

%% Calculate mean displacement for all these nodes, in x,y,z
d_x = mean(data(:,5) - data(:,2));
d_y = mean(data(:,6) - data(:,3));
d_z = mean(data(:,7) - data(:,4));
 
def = [d_x,d_y,d_z]


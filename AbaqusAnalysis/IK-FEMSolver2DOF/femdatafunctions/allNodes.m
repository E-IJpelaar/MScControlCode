function [all_nodes] = allNodes(filename)

% This function extracts all nodes of an entire part with a given
% lay-out. The current set up allows to extract node numbers that in
% Abaqus/CAE have been obtained by:
% 1) Tools>Query...>(sub menu "General Queries) "Node">(sub menu
% "Visualizatoin Module Queries) "Probe values"
% 2) In new diagolog > change "Probe" from "Elements" to "Nodes" > Select a
% display group > change "Display group from "<none>" to "All"
% 3) Let the Abaqus read out all nodes
% 4) Press "Write to file" in the same dialog

fid = fopen(filename,'rt') ;
S = textscan(fid,'%s','Delimiter','\n');
S = S{1} ;
%%Get the line number of mises 
idxNode = strfind(S, '--------------------'); % find this exppression
idx1 = find(not(cellfun('isempty', idxNode)));% find index of this expression
% pick  nodes 
allnodes = S(idx1(1)+1:idx1(2)-4);            % all nodes can be found between this expression
AllNodes = zeros(length(allnodes),7);
for ii = 1:length(allnodes)
    AllNodes(ii,:) = str2num(allnodes{ii}(15:end)); % write all data in matrix 
end
all_nodes = AllNodes;
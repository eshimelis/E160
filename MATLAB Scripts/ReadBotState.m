function [ robotState ] = ReadBotState( filename )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    robotState = readtable(filename);

end


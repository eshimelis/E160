function [ robotState ] = ReadBotState( filename )
%ReadBotState Reads robot state and logs output

    robotState = readtable(filename);

end


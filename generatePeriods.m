% notes         =  C^     , C      , C#/D^  , D     , D#/E^  , E/F^   , E#/F   , F#/G^  , G      , G#/A^  , A  , A#/B^  , B      , B#
noteFrequencies = [493.883, 523.251, 554.365, 587.33, 622.254, 659.255, 698.456, 739.989, 783.991, 830.609, 880, 932.328, 987.767, 1046.502];
% src : [http://www.liutaomottola.com/formulae/freqtab.htm]

% find periods of these frequencies
notePeriod = 1./noteFrequencies * 10^6;

% print ready for C++ code
fprintf('{')
for noteIndex = 1:length(noteFrequencies)
    fprintf('%.4f, // %.3f    %i    \n',notePeriod(noteIndex),noteFrequencies(noteIndex), noteIndex)
end
fprintf('}\n')

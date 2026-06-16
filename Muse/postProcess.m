function estimationResults = postProcess(estimationResults)
% postProcess - Post-process estimation results for FakeSpring model
% Depending on model some rescaling is needed. We use coupling forces as
% kN internally in the model, we need to convert back to N for output.

    fields = fieldnames(estimationResults);
    if any(strcmpi(fields, 'COUPLINGFORCE'))
        posi = fieldnames(estimationResults.COUPLINGFORCE);
        for i = 1:length(posi)
            estimationResults.COUPLINGFORCE.(posi{i}) = estimationResults.COUPLINGFORCE.(posi{i}) * 1000; % Convert kN to N
            estimationResults.uncertainty.COUPLINGFORCE.(posi{i}) = estimationResults.uncertainty.COUPLINGFORCE.(posi{i}) * 1000; % Convert kN to N
        end
    end
end

    % plot_dummies for legend
    hold on
l1 = plot([,NaN], 'r*');
l2 = plot([NaN,NaN], 'b*');
l3 = plot([NaN,NaN], 'g*');
l4 = plot([NaN,NaN], 'm*');
legend([l1, l2, l3, l4], {'significant - far from wall', 'significant - near wall', ...
    'not significant - far from wall', 'not significant - near wall'});
    
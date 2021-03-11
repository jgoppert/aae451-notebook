###############################################################################
# Debug display - stand in instrumentation.
var debug_display_view_handler = {
    init : func {
        if (contains(me, "left")) return;

        me.left  = screen.display.new(20, 10);
        me.right = screen.display.new(-200, 10);
        me.left.format  = "%.5g";
        me.right.format = "%.5g";
        me.left.add("/orientation/pitch-deg");
        me.left.add("/fdm/jsbsim/hydro/beta-deg");
        #me.left.add("/fdm/jsbsim/hydro/left-float-submersion-ft");
        #me.left.add("/fdm/jsbsim/hydro/right-float-submersion-ft");
        #me.left.add("/fdm/jsbsim/hydro/coefficients/C_V");
        #me.left.add("/fdm/jsbsim/hydro/coefficients/C_Delta");
        #me.left.add("/fdm/jsbsim/hydro/coefficients/C_R");
        #me.left.add("/fdm/jsbsim/hydro/coefficients/C_M");
        me.left.add("/fdm/jsbsim/inertia/cg-x-in");

        me.left.add("/fdm/jsbsim/electrical/bus[0]/voltage-V");
        me.left.add("/fdm/jsbsim/electrical/bus[0]/current-A");
        me.left.add("/fdm/jsbsim/electrical/bus[1]/voltage-V");
        me.left.add("/fdm/jsbsim/electrical/bus[1]/current-A");
        me.left.add("/fdm/jsbsim/electrical/battery/total-current-A");
        me.left.add("/fdm/jsbsim/electrical/voltage-regulators/current-A[0]");
        me.left.add("/fdm/jsbsim/electrical/voltage-regulators/current-A[1]");

        me.right.add("/fdm/jsbsim/propulsion/engine[0]/boost-psi");
        me.right.add("/fdm/jsbsim/propulsion/engine[1]/boost-psi");
        me.right.add("/fdm/jsbsim/propulsion/engine[2]/boost-psi");
        me.right.add("/fdm/jsbsim/propulsion/engine[3]/boost-psi");
        me.right.add("/engines/engine[0]/rpm");
        me.right.add("/engines/engine[1]/rpm");
        me.right.add("/engines/engine[2]/rpm");
        me.right.add("/engines/engine[3]/rpm");
        me.right.add("/fdm/jsbsim/propulsion/engine[0]/power-hp");
        me.right.add("/fdm/jsbsim/propulsion/engine[1]/power-hp");
        me.right.add("/fdm/jsbsim/propulsion/engine[2]/power-hp");
        me.right.add("/fdm/jsbsim/propulsion/engine[3]/power-hp");
        me.right.add("/fdm/jsbsim/propulsion/engine[0]/egt-degF");
        me.right.add("/fdm/jsbsim/propulsion/engine[1]/egt-degF");
        me.right.add("/fdm/jsbsim/propulsion/engine[2]/egt-degF");
        me.right.add("/fdm/jsbsim/propulsion/engine[3]/egt-degF");
        me.right.add("/fdm/jsbsim/fcs/fuel-system/debug/consumption-error-lbs");
        me.shown = 1;
        me.stop();
    },
    start  : func {
        if (!me.shown) {
            me.left.toggle();
            me.right.toggle();
        }
        me.shown = 1;
    },
    stop   : func {
        if (me.shown) {
            me.left.toggle();
            me.right.toggle();
        }
        me.shown = 0;
    }
};

# Install the debug display for some views.
setlistener("/sim/signals/fdm-initialized", func {
    view.manager.register(0, debug_display_view_handler);
});

# full library
# add_library(DspFilters Bessel.cpp Bessel.h Biquad.cpp Biquad.h Butterworth.cpp Butterworth.h Cascade.cpp Cascade.h ChebyshevI.cpp ChebyshevI.h ChebyshevII.cpp ChebyshevII.h Common.h Custom.cpp Custom.h Design.cpp Design.h Documentation.cpp Dsp.h Elliptic.cpp Elliptic.h Filter.cpp Filter.h Layout.h Legendre.cpp Legendre.h MathSupplement.h Param.cpp Params.h PoleFilter.cpp PoleFilter.h RBJ.cpp RBJ.h RootFinder.cpp RootFinder.h SmoothedFilter.h State.cpp State.h Types.h Utilities.h)

# ChebyshevI filter only
add_library(DspFilters Biquad.cpp Biquad.h Cascade.cpp Cascade.h ChebyshevI.cpp ChebyshevI.h Common.h Design.cpp Design.h Filter.cpp Filter.h Layout.h MathSupplement.h Params.h PoleFilter.cpp PoleFilter.h State.cpp State.h Types.h)

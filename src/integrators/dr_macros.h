#pragma once
/*---------------------------------------------------------------------------------------------*/
/* Adaptive multiview path tracing; Bc. Ondrej Ac, FIT VUT Brno, 2025*/
/*---------------------------------------------------------------------------------------------*/
#include <type_traits>

// Macros From: https://github.com/18sg/uSHET/blob/master/lib/cpp_magic.h

#define FIRST(a, ...) a
#define SECOND(a, b, ...) b

#define EMPTY()

#define EVAL(...) EVAL32(__VA_ARGS__)
#define EVAL32(...) EVAL16(EVAL16(__VA_ARGS__))
#define EVAL16(...) EVAL8(EVAL8(__VA_ARGS__))
#define EVAL8(...) EVAL4(EVAL4(__VA_ARGS__))
#define EVAL4(...) EVAL2(EVAL2(__VA_ARGS__))
#define EVAL2(...) EVAL1(EVAL1(__VA_ARGS__))
#define EVAL1(...) __VA_ARGS__

#define DEFER1(m) m EMPTY()
#define DEFER2(m) m EMPTY EMPTY()()
#define DEFER3(m) m EMPTY EMPTY EMPTY()()()
#define DEFER4(m) m EMPTY EMPTY EMPTY EMPTY()()()()

#define IS_PROBE(...) SECOND(__VA_ARGS__, 0)
#define PROBE() ~, 1

#define CAT(a,b) a ## b

#define NOT(x) IS_PROBE(CAT(_NOT_, x))
#define _NOT_0 PROBE()

#define BOOL(x) NOT(NOT(x))

#define IF_ELSE(condition) _IF_ELSE(BOOL(condition))
#define _IF_ELSE(condition) CAT(_IF_, condition)

#define _IF_1(...) __VA_ARGS__ _IF_1_ELSE
#define _IF_0(...)             _IF_0_ELSE

#define _IF_1_ELSE(...)
#define _IF_0_ELSE(...) __VA_ARGS__

#define HAS_ARGS(...) BOOL(FIRST(_END_OF_ARGUMENTS_ __VA_ARGS__)())
#define _END_OF_ARGUMENTS_() 0

#define MAP(m, first, ...)           \
  m(first)                           \
  IF_ELSE(HAS_ARGS(__VA_ARGS__))(    \
    , DEFER2(_MAP)()(m, __VA_ARGS__)   \
  )(                                 \
    /* Terminate */ \
  )
#define MAP_SEMI(m, first, ...)           \
  m(first)                           \
  IF_ELSE(HAS_ARGS(__VA_ARGS__))(    \
    ; DEFER2(_MAP_SEMI)()(m, __VA_ARGS__)   \
  )(                                 \
    /* Terminate */ \
  )
#define _MAP() MAP
#define _MAP_SEMI() MAP_SEMI
#define EXPAND(...) __VA_ARGS__

// AMVPT specific macros
template <typename T>
using baseType = std::remove_const_t<std::remove_reference_t<T>>;
#define DECL(x) const baseType<decltype(x)>& x
#define DECL_REF(x) decltype(x)& x

#define DECLVAL(x) std::decay<decltype(x)> x

#define MARK_USED(x) (void)x
// Mark parm list as used
#define SILENCE(...) EVAL(MAP(MARK_USED, __VA_ARGS__))
// Declare as variable list with types
#define DECLARE(...) EVAL(MAP(DECL, __VA_ARGS__))
// Non const version
#define DECLARE_REF(...) EVAL(MAP(DECL_REF, __VA_ARGS__))

// Macro for cleaner dr::if_stmt
#define DR_IF(cond, args, true_fn, false_fn) \
dr::if_stmt(std::make_tuple(EXPAND args), cond, [](DECLARE args){SILENCE(EXPAND args); EXPAND true_fn},  [](DECLARE args){SILENCE(EXPAND args); EXPAND false_fn})
// Macro for cleaner dr::while_loop
#define DR_LOOP(args, cond, body) \
dr::while_loop(std::make_tuple(EXPAND args), [](DECLARE args){SILENCE(EXPAND args); EXPAND cond}, [](DECLARE_REF args){SILENCE(EXPAND args); EXPAND body})

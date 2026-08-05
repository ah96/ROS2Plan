; Untyped PDDL fixture. There is no (:types) declaration, so the parser leaves the type
; pointer null on every constant, object and parameter it produces. Between them the
; :constants block, the :objects block of the companion problem, and the (distance ?from ?to)
; term in move's numeric effect cover each place the knowledge base reads a symbol's type.
(define (domain untyped_test)
 (:requirements :strips :fluents)

 (:constants depot)

 (:predicates
  (at ?r ?wp)
  (connected ?from ?to))

 (:functions
  (distance ?from ?to)
  (travelled))

 (:action move
   :parameters (?r ?from ?to)
   :precondition (and (at ?r ?from)
                      (connected ?from ?to))
   :effect (and (not (at ?r ?from))
                (at ?r ?to)
                (increase (travelled) (distance ?from ?to)))
 )
)

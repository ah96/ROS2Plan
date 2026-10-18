; Companion problem for the untyped domain. The :objects block has no "- type" suffixes, which
; is the path that used to segfault the knowledge base while it parsed the problem file.
(define (problem untyped_task)
 (:domain untyped_test)

 (:objects robot wpa wpb)

 (:init
  (at robot wpa)
  (connected depot wpa)
  (connected wpa wpb)
  (connected wpb depot)
  (= (distance depot wpa) 2)
  (= (distance wpa wpb) 3)
  (= (distance wpb depot) 4)
  (= (travelled) 0))

 (:goal (and
  (at robot wpb)))
)

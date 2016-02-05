(define (problem cob-test-problem-01)
 	(:domain cob-test-domain-01)
 	(:objects 
  		 ;;; available objects 
  
  		 ;;; fixed locations 
 		 room-x-sq-0 room-x-sq-1 room-x-sq-2 room-x-sq-3 
 		 room-x-sq-4 room-x-sq-5 room-x-sq-6 room-x-sq-7 
 		 room-x-sq-8 room-x-sq-9 room-x-sq-10 room-x-sq-11 
 		 room-x-sq-12 room-x-sq-13 room-x-sq-14 
  
  		 ;;; movable things 
  
 		 the-cake 
 		 cob4-1)
 
 
 	(:init  
  		 ;;; transitions for room-x-sq-0
 		(trans room-x-sq-0 room-x-sq-1)
 		(trans room-x-sq-0 room-x-sq-9)
 
  		 ;;; transitions for room-x-sq-1
 		(trans room-x-sq-1 room-x-sq-0)
 		(trans room-x-sq-1 room-x-sq-2)
 		(trans room-x-sq-1 room-x-sq-8)
 
  		 ;;; transitions for room-x-sq-2
 		(trans room-x-sq-2 room-x-sq-3)
 		(trans room-x-sq-2 room-x-sq-1)
 		(trans room-x-sq-2 room-x-sq-7)
 
  		 ;;; transitions for room-x-sq-3
 		(trans room-x-sq-3 room-x-sq-2)
 		(trans room-x-sq-3 room-x-sq-4)
 		(trans room-x-sq-3 room-x-sq-6)
 
  		 ;;; transitions for room-x-sq-4
 		(trans room-x-sq-4 room-x-sq-3)
 		(trans room-x-sq-4 room-x-sq-5)
 
  		 ;;; transitions for room-x-sq-5
 		(trans room-x-sq-5 room-x-sq-6)
 		(trans room-x-sq-5 room-x-sq-4)
 		(trans room-x-sq-5 room-x-sq-10)
 
  		 ;;; transitions for room-x-sq-6
 		(trans room-x-sq-6 room-x-sq-5)
 		(trans room-x-sq-6 room-x-sq-7)
 		(trans room-x-sq-6 room-x-sq-3)
 		(trans room-x-sq-6 room-x-sq-11)
 
  		 ;;; transitions for room-x-sq-7
 		(trans room-x-sq-7 room-x-sq-6)
 		(trans room-x-sq-7 room-x-sq-2)
 		(trans room-x-sq-7 room-x-sq-8)
 		(trans room-x-sq-7 room-x-sq-12)
 
  		 ;;; transitions for room-x-sq-8
 		(trans room-x-sq-8 room-x-sq-9)
 		(trans room-x-sq-8 room-x-sq-1)
 		(trans room-x-sq-8 room-x-sq-7)
 		(trans room-x-sq-8 room-x-sq-13)
 
  		 ;;; transitions for room-x-sq-9
 		(trans room-x-sq-9 room-x-sq-8)
 		(trans room-x-sq-9 room-x-sq-0)
 		(trans room-x-sq-9 room-x-sq-14)
 
  		 ;;; transitions for room-x-sq-10
 		(trans room-x-sq-10 room-x-sq-11)
 		(trans room-x-sq-10 room-x-sq-5)
 
  		 ;;; transitions for room-x-sq-11
 		(trans room-x-sq-11 room-x-sq-10)
 		(trans room-x-sq-11 room-x-sq-12)
 		(trans room-x-sq-11 room-x-sq-6)
 
  		 ;;; transitions for room-x-sq-12
 		(trans room-x-sq-12 room-x-sq-11)
 		(trans room-x-sq-12 room-x-sq-13)
 		(trans room-x-sq-12 room-x-sq-7)
 
  		 ;;; transitions for room-x-sq-13
 		(trans room-x-sq-13 room-x-sq-12)
 		(trans room-x-sq-13 room-x-sq-14)
 		(trans room-x-sq-13 room-x-sq-8)
 
  		 ;;; transitions for room-x-sq-14
 		(trans room-x-sq-14 room-x-sq-13)
 		(trans room-x-sq-14 room-x-sq-9)
 
 	 ;;; hard coded definitions
 		(is-robo cob4-1)
 		(at the-cake room-x-sq-10)
 		(at cob4-1 room-x-sq-0)
 	)
 
 
 	;;; goal definition
 	(:goal (and (have cob4-1 the-cake) (at cob4-1 room-x-sq-0)))
 )

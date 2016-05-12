(define (problem cob-test-problem-01)
	(:domain cob-test-domain-01)
	(:objects 
 		 ;;; available objects 
 
 		 ;;; fixed locations 
		 room-1 room-2 room-3 room-4 room-5
		 room-6 room-7 room-8 room-9 room-10
		 room-11 room-12 room-13
 
		 ;;; fixed things for interaction
		 ;;;arm-left
		 ;;;arm-right
		 ;;;user
		 ;;;door-1
 		 ;;; movable things 
 		
		 ;;;the-cake 
		 cob4-1
		 ;;;the-box
		)


	(:init  
 		 ;;; transitions for room-1
		(trans room-1 room-2)

 		 ;;; transitions for room-2
		(trans room-2 room-1)
		(trans room-2 room-6)
		(trans room-2 room-3)

 		 ;;; transitions for room-3
		(trans room-3 room-2)

 		 ;;; transitions for room-4
		(trans room-4 room-6)

 		 ;;; transitions for room-5
		(trans room-5 room-6)

 		 ;;; transitions for room-6
		(trans room-6 room-5)
		(trans room-6 room-9)
		(trans room-6 room-2)
		(trans room-6 room-7)
		(trans room-6 room-4)

 		 ;;; transitions for room-7
		(trans room-7 room-6)
		(trans room-7 room-8)

 		 ;;; transitions for room-8
		(trans room-8 room-7)
		(trans room-8 room-12)
		(trans room-8 room-11)

 		 ;;; transitions for room-9
		(trans room-9 room-6)
		(trans room-9 room-13)
		(trans room-9 room-12)

 		 ;;; transitions for room-10
		(trans room-10 room-12)

 		 ;;; transitions for room-11
		(trans room-11 room-8)

 		 ;;; transitions for room-12
		(trans room-12 room-9)
		(trans room-12 room-8)
		(trans room-12 room-10)

 		 ;;; transitions for room-13
		(trans room-13 room-9)

	 ;;; hard coded definitions
		(is-robo cob4-1)
		(at cob4-1 room-1)
		
		
	)


	;;; goal definition
	(:goal (and (cleaned room-4)))
	;;;(:goal (and (at the-cake room-12-square-7)))
	;;;(:goal (and (have cob4-1 the-cake) (at cob4-1 room-12-square-7)))
)

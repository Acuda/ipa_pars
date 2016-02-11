(define (problem cob-test-problem-01)
	(:domain cob-test-domain-01)
	(:objects 
 		 ;;; available objects 
 
 		 ;;; fixed locations 
		 room-7 room-5 room-1 room-6 
		 room-2 room-4 room-3 room-9 
		 room-8 
 
 		 ;;; movable things 
 
		 the-cake 
		 cob4-1)


	(:init  
 		 ;;; transitions for room-7
		(trans room-7 room-6)

 		 ;;; transitions for room-5
		(trans room-5 room-2)

 		 ;;; transitions for room-1
		(trans room-1 room-2)

 		 ;;; transitions for room-6
		(trans room-6 room-2)
		(trans room-6 room-7)
		(trans room-6 room-8)

 		 ;;; transitions for room-2
		(trans room-2 room-5)
		(trans room-2 room-1)
		(trans room-2 room-6)
		(trans room-2 room-4)
		(trans room-2 room-3)

 		 ;;; transitions for room-4
		(trans room-4 room-2)

 		 ;;; transitions for room-3
		(trans room-3 room-2)

 		 ;;; transitions for room-9
		(trans room-9 room-8)

 		 ;;; transitions for room-8
		(trans room-8 room-9)
		(trans room-8 room-6)

	 ;;; hard coded definitions
		(is-robo cob4-1)
		(at the-cake room-9)
		(at cob4-1 room-1)
	)


	;;; goal definition
	(:goal (and (have cob4-1 the-cake) (at cob4-1 room-1)))
)
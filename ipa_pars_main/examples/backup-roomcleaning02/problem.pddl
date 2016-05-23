(define (problem cob-test-problem-01)
	(:domain cob-test-domain-01)
	(:objects 
 		;;; available objects 
 
 		;;; fixed locations 
		room-1 room-2 room-3 - room
 
		;;; fixed things for interaction
 		
		;;; dynamically added trash-bins:
		trash-bin-1 trash-bin-2 trash-bin-3 trash-bin-4 - trash-bin
		dirt-1 dirt-2 dirt-3 dirt-4 - dirt 
		cob4-1 - robot
		)


	(:init  
 		 ;;; transitions for room-1
		(trans room-1 room-2)

 		 ;;; transitions for room-2
		(trans room-2 room-1)
		(trans room-2 room-3)

 		 ;;; transitions for room-3
		(trans room-3 room-2)

	 ;;; hard coded definitions
		;;; robot location
		(at cob4-1 room-1)

		;;; trash-bin location
		(at trash-bin-1 room-1)
		(at trash-bin-2 room-3)
		(at trash-bin-3 room-3)

		;;; dirtmap entries
		(at dirt-1 room-1)
		(at dirt-2 room-1)
		(at dirt-3 room-3)
	)


	;;; goal definition
	(:goal (and (forall (?r - room) (inspected ?r) 
		    )
		    (forall (?t - trash-bin) (cleared ?t)
		    )
		    (forall (?d - dirt) (cleaned ?d)
		    )
		)
	)
)

import re

# Text containing the BFS times
bfs_text = """
TEPS for BFS 3461 is 2.07385e+08
Running BFS 3462
Current Time of Running BFS: Mon Jan  8 15:48:19 2024

Time for BFS 3462 is 0.078655
Current Time of Time for BFS: Mon Jan  8 15:48:19 2024

TEPS for BFS 3462 is 2.13283e+08
Running BFS 3463
Current Time of Running BFS: Mon Jan  8 15:48:19 2024

Time for BFS 3463 is 0.075707
Current Time of Time for BFS: Mon Jan  8 15:48:19 2024

TEPS for BFS 3463 is 2.21589e+08
Running BFS 3464
Current Time of Running BFS: Mon Jan  8 15:48:19 2024

Time for BFS 3464 is 0.080355
Current Time of Time for BFS: Mon Jan  8 15:48:19 2024

TEPS for BFS 3464 is 2.08772e+08
Running BFS 3465
Current Time of Running BFS: Mon Jan  8 15:48:19 2024

Time for BFS 3465 is 0.083251
Current Time of Time for BFS: Mon Jan  8 15:48:19 2024

TEPS for BFS 3465 is 2.01509e+08
Running BFS 3466
Current Time of Running BFS: Mon Jan  8 15:48:19 2024

Time for BFS 3466 is 0.077921
Current Time of Time for BFS: Mon Jan  8 15:48:19 2024

TEPS for BFS 3466 is 2.15293e+08
Running BFS 3467
Current Time of Running BFS: Mon Jan  8 15:48:19 2024

Time for BFS 3467 is 0.080849
Current Time of Time for BFS: Mon Jan  8 15:48:19 2024

TEPS for BFS 3467 is 2.07497e+08
Running BFS 3468
Current Time of Running BFS: Mon Jan  8 15:48:19 2024

Time for BFS 3468 is 0.078281
Current Time of Time for BFS: Mon Jan  8 15:48:19 2024

TEPS for BFS 3468 is 2.14302e+08
Running BFS 3469
Current Time of Running BFS: Mon Jan  8 15:48:19 2024

Time for BFS 3469 is 0.082421
Current Time of Time for BFS: Mon Jan  8 15:48:19 2024

TEPS for BFS 3469 is 2.03539e+08
Running BFS 3470
Current Time of Running BFS: Mon Jan  8 15:48:19 2024

Time for BFS 3470 is 0.080106
Current Time of Time for BFS: Mon Jan  8 15:48:20 2024

TEPS for BFS 3470 is 2.09419e+08
Running BFS 3471
Current Time of Running BFS: Mon Jan  8 15:48:20 2024

Time for BFS 3471 is 0.084507
Current Time of Time for BFS: Mon Jan  8 15:48:20 2024

TEPS for BFS 3471 is 1.98513e+08
Running BFS 3472
Current Time of Running BFS: Mon Jan  8 15:48:20 2024

Time for BFS 3472 is 0.083453
Current Time of Time for BFS: Mon Jan  8 15:48:20 2024

TEPS for BFS 3472 is 2.01021e+08
Running BFS 3473
Current Time of Running BFS: Mon Jan  8 15:48:20 2024

Time for BFS 3473 is 0.080642
Current Time of Time for BFS: Mon Jan  8 15:48:20 2024

TEPS for BFS 3473 is 2.08029e+08
Running BFS 3474
Current Time of Running BFS: Mon Jan  8 15:48:20 2024

Time for BFS 3474 is 0.082452
Current Time of Time for BFS: Mon Jan  8 15:48:20 2024

TEPS for BFS 3474 is 2.03461e+08
Running BFS 3475
Current Time of Running BFS: Mon Jan  8 15:48:20 2024

Time for BFS 3475 is 0.077920
Current Time of Time for BFS: Mon Jan  8 15:48:20 2024

TEPS for BFS 3475 is 2.15295e+08
Running BFS 3476
Current Time of Running BFS: Mon Jan  8 15:48:20 2024

Time for BFS 3476 is 0.058326
Current Time of Time for BFS: Mon Jan  8 15:48:20 2024

TEPS for BFS 3476 is 2.87622e+08
Running BFS 3477
Current Time of Running BFS: Mon Jan  8 15:48:20 2024

Time for BFS 3477 is 0.054341
Current Time of Time for BFS: Mon Jan  8 15:48:20 2024

TEPS for BFS 3477 is 3.08715e+08
Running BFS 3478
Current Time of Running BFS: Mon Jan  8 15:48:20 2024

Time for BFS 3478 is 0.088984
Current Time of Time for BFS: Mon Jan  8 15:48:20 2024

TEPS for BFS 3478 is 1.88527e+08
Running BFS 3479
Current Time of Running BFS: Mon Jan  8 15:48:20 2024

Time for BFS 3479 is 0.099120
Current Time of Time for BFS: Mon Jan  8 15:48:20 2024

TEPS for BFS 3479 is 1.69248e+08
Running BFS 3480
Current Time of Running BFS: Mon Jan  8 15:48:20 2024

Time for BFS 3480 is 0.103846
Current Time of Time for BFS: Mon Jan  8 15:48:20 2024

TEPS for BFS 3480 is 1.61545e+08
Running BFS 3481
Current Time of Running BFS: Mon Jan  8 15:48:20 2024

Time for BFS 3481 is 0.128991
Current Time of Time for BFS: Mon Jan  8 15:48:21 2024

TEPS for BFS 3481 is 1.30055e+08
Running BFS 3482
Current Time of Running BFS: Mon Jan  8 15:48:21 2024

Time for BFS 3482 is 0.126867
Current Time of Time for BFS: Mon Jan  8 15:48:21 2024

TEPS for BFS 3482 is 1.32232e+08
Running BFS 3483
Current Time of Running BFS: Mon Jan  8 15:48:21 2024

Time for BFS 3483 is 0.097814
Current Time of Time for BFS: Mon Jan  8 15:48:21 2024

TEPS for BFS 3483 is 1.71507e+08
Running BFS 3484
Current Time of Running BFS: Mon Jan  8 15:48:21 2024

Time for BFS 3484 is 0.095340
Current Time of Time for BFS: Mon Jan  8 15:48:21 2024

TEPS for BFS 3484 is 1.75958e+08
Running BFS 3485
Current Time of Running BFS: Mon Jan  8 15:48:21 2024

Time for BFS 3485 is 0.109378
Current Time of Time for BFS: Mon Jan  8 15:48:21 2024

TEPS for BFS 3485 is 1.53375e+08
Running BFS 3486
Current Time of Running BFS: Mon Jan  8 15:48:21 2024

Time for BFS 3486 is 0.118599
Current Time of Time for BFS: Mon Jan  8 15:48:21 2024

TEPS for BFS 3486 is 1.4145e+08
Running BFS 3487
Current Time of Running BFS: Mon Jan  8 15:48:21 2024

Time for BFS 3487 is 0.119254
Current Time of Time for BFS: Mon Jan  8 15:48:21 2024

TEPS for BFS 3487 is 1.40673e+08
Running BFS 3488
Current Time of Running BFS: Mon Jan  8 15:48:21 2024

Time for BFS 3488 is 0.080448
Current Time of Time for BFS: Mon Jan  8 15:48:21 2024

TEPS for BFS 3488 is 2.08529e+08
Running BFS 3489
Current Time of Running BFS: Mon Jan  8 15:48:21 2024

Time for BFS 3489 is 0.080016
Current Time of Time for BFS: Mon Jan  8 15:48:22 2024

TEPS for BFS 3489 is 2.09655e+08
Running BFS 3490
Current Time of Running BFS: Mon Jan  8 15:48:22 2024

Time for BFS 3490 is 0.094716
Current Time of Time for BFS: Mon Jan  8 15:48:22 2024

TEPS for BFS 3490 is 1.77116e+08
Running BFS 3491
Current Time of Running BFS: Mon Jan  8 15:48:22 2024

Time for BFS 3491 is 0.096025
Current Time of Time for BFS: Mon Jan  8 15:48:22 2024

TEPS for BFS 3491 is 1.74703e+08
Running BFS 3492
Current Time of Running BFS: Mon Jan  8 15:48:22 2024

Time for BFS 3492 is 0.125829
Current Time of Time for BFS: Mon Jan  8 15:48:22 2024

TEPS for BFS 3492 is 1.33322e+08
Running BFS 3493
Current Time of Running BFS: Mon Jan  8 15:48:22 2024

Time for BFS 3493 is 0.124254
Current Time of Time for BFS: Mon Jan  8 15:48:22 2024

TEPS for BFS 3493 is 1.35012e+08
Running BFS 3494
Current Time of Running BFS: Mon Jan  8 15:48:22 2024

Time for BFS 3494 is 0.094638
Current Time of Time for BFS: Mon Jan  8 15:48:22 2024

TEPS for BFS 3494 is 1.77262e+08
Running BFS 3495
Current Time of Running BFS: Mon Jan  8 15:48:22 2024

Time for BFS 3495 is 0.067738
Current Time of Time for BFS: Mon Jan  8 15:48:22 2024

TEPS for BFS 3495 is 2.47659e+08
Running BFS 3496
Current Time of Running BFS: Mon Jan  8 15:48:22 2024

Time for BFS 3496 is 0.068188
Current Time of Time for BFS: Mon Jan  8 15:48:22 2024

TEPS for BFS 3496 is 2.46022e+08
Running BFS 3497
Current Time of Running BFS: Mon Jan  8 15:48:22 2024

Time for BFS 3497 is 0.063814
Current Time of Time for BFS: Mon Jan  8 15:48:22 2024

TEPS for BFS 3497 is 2.62884e+08
Running BFS 3498
Current Time of Running BFS: Mon Jan  8 15:48:22 2024

Time for BFS 3498 is 0.069360
Current Time of Time for BFS: Mon Jan  8 15:48:22 2024

TEPS for BFS 3498 is 2.41867e+08
Running BFS 3499
Current Time of Running BFS: Mon Jan  8 15:48:22 2024

Time for BFS 3499 is 0.068634
Current Time of Time for BFS: Mon Jan  8 15:48:23 2024

TEPS for BFS 3499 is 2.44426e+08
Running BFS 3500
Current Time of Running BFS: Mon Jan  8 15:48:23 2024

Time for BFS 3500 is 0.057023
Current Time of Time for BFS: Mon Jan  8 15:48:23 2024

TEPS for BFS 3500 is 2.94196e+08
Running BFS 3501
Current Time of Running BFS: Mon Jan  8 15:48:23 2024

Time for BFS 3501 is 0.066395
Current Time of Time for BFS: Mon Jan  8 15:48:23 2024

TEPS for BFS 3501 is 2.52667e+08
Running BFS 3502
Current Time of Running BFS: Mon Jan  8 15:48:23 2024

Time for BFS 3502 is 0.071884
Current Time of Time for BFS: Mon Jan  8 15:48:23 2024

TEPS for BFS 3502 is 2.33372e+08
Running BFS 3503
Current Time of Running BFS: Mon Jan  8 15:48:23 2024

Time for BFS 3503 is 0.100502
Current Time of Time for BFS: Mon Jan  8 15:48:23 2024

TEPS for BFS 3503 is 1.6692e+08
Running BFS 3504
Current Time of Running BFS: Mon Jan  8 15:48:23 2024

Time for BFS 3504 is 0.077503
Current Time of Time for BFS: Mon Jan  8 15:48:23 2024

TEPS for BFS 3504 is 2.16452e+08
Running BFS 3505
Current Time of Running BFS: Mon Jan  8 15:48:23 2024

Time for BFS 3505 is 0.070790
Current Time of Time for BFS: Mon Jan  8 15:48:23 2024

TEPS for BFS 3505 is 2.36981e+08
Running BFS 3506
Current Time of Running BFS: Mon Jan  8 15:48:23 2024

Time for BFS 3506 is 0.080302
Current Time of Time for BFS: Mon Jan  8 15:48:23 2024

TEPS for BFS 3506 is 2.08908e+08
Running BFS 3507
Current Time of Running BFS: Mon Jan  8 15:48:23 2024

Time for BFS 3507 is 0.075638
Current Time of Time for BFS: Mon Jan  8 15:48:23 2024

TEPS for BFS 3507 is 2.21791e+08
Running BFS 3508
Current Time of Running BFS: Mon Jan  8 15:48:23 2024

Time for BFS 3508 is 0.075561
Current Time of Time for BFS: Mon Jan  8 15:48:23 2024

TEPS for BFS 3508 is 2.22017e+08
Running BFS 3509
Current Time of Running BFS: Mon Jan  8 15:48:23 2024

Time for BFS 3509 is 0.076119
Current Time of Time for BFS: Mon Jan  8 15:48:23 2024

TEPS for BFS 3509 is 2.20391e+08
Running BFS 3510
Current Time of Running BFS: Mon Jan  8 15:48:23 2024

Time for BFS 3510 is 0.072312
Current Time of Time for BFS: Mon Jan  8 15:48:24 2024

TEPS for BFS 3510 is 2.31991e+08
Running BFS 3511
Current Time of Running BFS: Mon Jan  8 15:48:24 2024

Time for BFS 3511 is 0.072864
Current Time of Time for BFS: Mon Jan  8 15:48:24 2024

TEPS for BFS 3511 is 2.30233e+08
Running BFS 3512
Current Time of Running BFS: Mon Jan  8 15:48:24 2024

Time for BFS 3512 is 0.078210
Current Time of Time for BFS: Mon Jan  8 15:48:24 2024

TEPS for BFS 3512 is 2.14496e+08
Running BFS 3513
Current Time of Running BFS: Mon Jan  8 15:48:24 2024

Time for BFS 3513 is 0.084467
Current Time of Time for BFS: Mon Jan  8 15:48:24 2024

TEPS for BFS 3513 is 1.98609e+08
Running BFS 3514
Current Time of Running BFS: Mon Jan  8 15:48:24 2024

Time for BFS 3514 is 0.079863
Current Time of Time for BFS: Mon Jan  8 15:48:24 2024

TEPS for BFS 3514 is 2.10058e+08
Running BFS 3515
Current Time of Running BFS: Mon Jan  8 15:48:24 2024

Time for BFS 3515 is 0.076831
Current Time of Time for BFS: Mon Jan  8 15:48:24 2024

TEPS for BFS 3515 is 2.18348e+08
Running BFS 3516
Current Time of Running BFS: Mon Jan  8 15:48:24 2024

Time for BFS 3516 is 0.078224
Current Time of Time for BFS: Mon Jan  8 15:48:24 2024

TEPS for BFS 3516 is 2.14459e+08
Running BFS 3517
Current Time of Running BFS: Mon Jan  8 15:48:24 2024

Time for BFS 3517 is 0.073728
Current Time of Time for BFS: Mon Jan  8 15:48:24 2024

TEPS for BFS 3517 is 2.27535e+08
Running BFS 3518
Current Time of Running BFS: Mon Jan  8 15:48:24 2024

Time for BFS 3518 is 0.069751
Current Time of Time for BFS: Mon Jan  8 15:48:24 2024

TEPS for BFS 3518 is 2.4051e+08
Running BFS 3519
Current Time of Running BFS: Mon Jan  8 15:48:24 2024

Time for BFS 3519 is 0.071838
Current Time of Time for BFS: Mon Jan  8 15:48:24 2024

TEPS for BFS 3519 is 2.33522e+08
Running BFS 3520
Current Time of Running BFS: Mon Jan  8 15:48:24 2024

Time for BFS 3520 is 0.112142
Current Time of Time for BFS: Mon Jan  8 15:48:24 2024

TEPS for BFS 3520 is 1.49594e+08
Running BFS 3521
Current Time of Running BFS: Mon Jan  8 15:48:24 2024

Time for BFS 3521 is 0.097346
Current Time of Time for BFS: Mon Jan  8 15:48:25 2024

TEPS for BFS 3521 is 1.72332e+08
Running BFS 3522
Current Time of Running BFS: Mon Jan  8 15:48:25 2024

Time for BFS 3522 is 0.093151
Current Time of Time for BFS: Mon Jan  8 15:48:25 2024

TEPS for BFS 3522 is 1.80093e+08
Running BFS 3523
Current Time of Running BFS: Mon Jan  8 15:48:25 2024

Time for BFS 3523 is 0.075300
Current Time of Time for BFS: Mon Jan  8 15:48:25 2024

TEPS for BFS 3523 is 2.22787e+08
Running BFS 3524
Current Time of Running BFS: Mon Jan  8 15:48:25 2024

Time for BFS 3524 is 0.079961
Current Time of Time for BFS: Mon Jan  8 15:48:25 2024

TEPS for BFS 3524 is 2.09799e+08
Running BFS 3525
Current Time of Running BFS: Mon Jan  8 15:48:25 2024

Time for BFS 3525 is 0.078657
Current Time of Time for BFS: Mon Jan  8 15:48:25 2024

TEPS for BFS 3525 is 2.13277e+08
Running BFS 3526
Current Time of Running BFS: Mon Jan  8 15:48:25 2024

Time for BFS 3526 is 0.077834
Current Time of Time for BFS: Mon Jan  8 15:48:25 2024

TEPS for BFS 3526 is 2.15532e+08
Running BFS 3527
Current Time of Running BFS: Mon Jan  8 15:48:25 2024

Time for BFS 3527 is 0.074475
Current Time of Time for BFS: Mon Jan  8 15:48:25 2024

TEPS for BFS 3527 is 2.25253e+08
Running BFS 3528
Current Time of Running BFS: Mon Jan  8 15:48:25 2024

Time for BFS 3528 is 0.076215
Current Time of Time for BFS: Mon Jan  8 15:48:25 2024

TEPS for BFS 3528 is 2.20111e+08
Running BFS 3529
Current Time of Running BFS: Mon Jan  8 15:48:25 2024

Time for BFS 3529 is 0.076199
Current Time of Time for BFS: Mon Jan  8 15:48:25 2024

TEPS for BFS 3529 is 2.20159e+08
Running BFS 3530
Current Time of Running BFS: Mon Jan  8 15:48:25 2024

Time for BFS 3530 is 0.076461
Current Time of Time for BFS: Mon Jan  8 15:48:25 2024

TEPS for BFS 3530 is 2.19402e+08
Running BFS 3531
Current Time of Running BFS: Mon Jan  8 15:48:25 2024

Time for BFS 3531 is 0.075599
Current Time of Time for BFS: Mon Jan  8 15:48:25 2024

TEPS for BFS 3531 is 2.21905e+08
Running BFS 3532
Current Time of Running BFS: Mon Jan  8 15:48:25 2024

Time for BFS 3532 is 0.074607
Current Time of Time for BFS: Mon Jan  8 15:48:26 2024

TEPS for BFS 3532 is 2.24856e+08
Running BFS 3533
Current Time of Running BFS: Mon Jan  8 15:48:26 2024

Time for BFS 3533 is 0.078534
Current Time of Time for BFS: Mon Jan  8 15:48:26 2024

TEPS for BFS 3533 is 2.13611e+08
Running BFS 3534
Current Time of Running BFS: Mon Jan  8 15:48:26 2024

Time for BFS 3534 is 0.080073
Current Time of Time for BFS: Mon Jan  8 15:48:26 2024

TEPS for BFS 3534 is 2.09505e+08
Running BFS 3535
Current Time of Running BFS: Mon Jan  8 15:48:26 2024

Time for BFS 3535 is 0.074829
Current Time of Time for BFS: Mon Jan  8 15:48:26 2024

TEPS for BFS 3535 is 2.24188e+08
Running BFS 3536
Current Time of Running BFS: Mon Jan  8 15:48:26 2024

Time for BFS 3536 is 0.074494
Current Time of Time for BFS: Mon Jan  8 15:48:26 2024

TEPS for BFS 3536 is 2.25196e+08
Running BFS 3537
Current Time of Running BFS: Mon Jan  8 15:48:26 2024

Time for BFS 3537 is 0.071716
Current Time of Time for BFS: Mon Jan  8 15:48:26 2024

TEPS for BFS 3537 is 2.3392e+08
Running BFS 3538
Current Time of Running BFS: Mon Jan  8 15:48:26 2024

Time for BFS 3538 is 0.074895
Current Time of Time for BFS: Mon Jan  8 15:48:26 2024

TEPS for BFS 3538 is 2.2399e+08
Running BFS 3539
Current Time of Running BFS: Mon Jan  8 15:48:26 2024

Time for BFS 3539 is 0.074800
Current Time of Time for BFS: Mon Jan  8 15:48:26 2024

TEPS for BFS 3539 is 2.24277e+08
Running BFS 3540
Current Time of Running BFS: Mon Jan  8 15:48:26 2024

Time for BFS 3540 is 0.078446
Current Time of Time for BFS: Mon Jan  8 15:48:26 2024

TEPS for BFS 3540 is 2.13852e+08
Running BFS 3541
Current Time of Running BFS: Mon Jan  8 15:48:26 2024

Time for BFS 3541 is 0.075887
Current Time of Time for BFS: Mon Jan  8 15:48:26 2024

TEPS for BFS 3541 is 2.21063e+08
Running BFS 3542
Current Time of Running BFS: Mon Jan  8 15:48:26 2024

Time for BFS 3542 is 0.079647
Current Time of Time for BFS: Mon Jan  8 15:48:26 2024

TEPS for BFS 3542 is 2.10627e+08
Running BFS 3543
Current Time of Running BFS: Mon Jan  8 15:48:26 2024

Time for BFS 3543 is 0.072402
Current Time of Time for BFS: Mon Jan  8 15:48:27 2024

TEPS for BFS 3543 is 2.31705e+08
Running BFS 3544
Current Time of Running BFS: Mon Jan  8 15:48:27 2024

Time for BFS 3544 is 0.075493
Current Time of Time for BFS: Mon Jan  8 15:48:27 2024

TEPS for BFS 3544 is 2.22218e+08
Running BFS 3545
Current Time of Running BFS: Mon Jan  8 15:48:27 2024

Time for BFS 3545 is 0.077891
Current Time of Time for BFS: Mon Jan  8 15:48:27 2024

TEPS for BFS 3545 is 2.15376e+08
Running BFS 3546
Current Time of Running BFS: Mon Jan  8 15:48:27 2024

Time for BFS 3546 is 0.074122
Current Time of Time for BFS: Mon Jan  8 15:48:27 2024

TEPS for BFS 3546 is 2.26328e+08
Running BFS 3547
Current Time of Running BFS: Mon Jan  8 15:48:27 2024

Time for BFS 3547 is 0.078521
Current Time of Time for BFS: Mon Jan  8 15:48:27 2024

TEPS for BFS 3547 is 2.13646e+08
Running BFS 3548
Current Time of Running BFS: Mon Jan  8 15:48:27 2024

Time for BFS 3548 is 0.077637
Current Time of Time for BFS: Mon Jan  8 15:48:27 2024

TEPS for BFS 3548 is 2.16081e+08
Running BFS 3549
Current Time of Running BFS: Mon Jan  8 15:48:27 2024

Time for BFS 3549 is 0.075344
Current Time of Time for BFS: Mon Jan  8 15:48:27 2024

TEPS for BFS 3549 is 2.22656e+08
Running BFS 3550
Current Time of Running BFS: Mon Jan  8 15:48:27 2024

Time for BFS 3550 is 0.072064
Current Time of Time for BFS: Mon Jan  8 15:48:27 2024

TEPS for BFS 3550 is 2.3279e+08
Running BFS 3551
Current Time of Running BFS: Mon Jan  8 15:48:27 2024

Time for BFS 3551 is 0.074234
Current Time of Time for BFS: Mon Jan  8 15:48:27 2024

TEPS for BFS 3551 is 2.25986e+08
Running BFS 3552
Current Time of Running BFS: Mon Jan  8 15:48:27 2024

Time for BFS 3552 is 0.075776
Current Time of Time for BFS: Mon Jan  8 15:48:27 2024

TEPS for BFS 3552 is 2.21386e+08
Running BFS 3553
Current Time of Running BFS: Mon Jan  8 15:48:27 2024

Time for BFS 3553 is 0.072400
Current Time of Time for BFS: Mon Jan  8 15:48:27 2024

TEPS for BFS 3553 is 2.31709e+08
Running BFS 3554
Current Time of Running BFS: Mon Jan  8 15:48:27 2024

Time for BFS 3554 is 0.075049
Current Time of Time for BFS: Mon Jan  8 15:48:28 2024

TEPS for BFS 3554 is 2.2353e+08
Running BFS 3555
Current Time of Running BFS: Mon Jan  8 15:48:28 2024

Time for BFS 3555 is 0.077001
Current Time of Time for BFS: Mon Jan  8 15:48:28 2024

TEPS for BFS 3555 is 2.17865e+08
Running BFS 3556
Current Time of Running BFS: Mon Jan  8 15:48:28 2024

Time for BFS 3556 is 0.073118
Current Time of Time for BFS: Mon Jan  8 15:48:28 2024

TEPS for BFS 3556 is 2.29435e+08
Running BFS 3557
Current Time of Running BFS: Mon Jan  8 15:48:28 2024

Time for BFS 3557 is 0.073886
Current Time of Time for BFS: Mon Jan  8 15:48:28 2024

TEPS for BFS 3557 is 2.2705e+08
Running BFS 3558
Current Time of Running BFS: Mon Jan  8 15:48:28 2024

Time for BFS 3558 is 0.075734
Current Time of Time for BFS: Mon Jan  8 15:48:28 2024

TEPS for BFS 3558 is 2.2151e+08
Running BFS 3559
Current Time of Running BFS: Mon Jan  8 15:48:28 2024

Time for BFS 3559 is 0.074017
Current Time of Time for BFS: Mon Jan  8 15:48:28 2024

TEPS for BFS 3559 is 2.26648e+08
Running BFS 3560
Current Time of Running BFS: Mon Jan  8 15:48:28 2024

Time for BFS 3560 is 0.074994
Current Time of Time for BFS: Mon Jan  8 15:48:28 2024

TEPS for BFS 3560 is 2.23696e+08
Running BFS 3561
Current Time of Running BFS: Mon Jan  8 15:48:28 2024

Time for BFS 3561 is 0.076974
Current Time of Time for BFS: Mon Jan  8 15:48:28 2024

TEPS for BFS 3561 is 2.17941e+08
Running BFS 3562
Current Time of Running BFS: Mon Jan  8 15:48:28 2024

Time for BFS 3562 is 0.080062
Current Time of Time for BFS: Mon Jan  8 15:48:28 2024

TEPS for BFS 3562 is 2.09535e+08
Running BFS 3563
Current Time of Running BFS: Mon Jan  8 15:48:28 2024

Time for BFS 3563 is 0.076144
Current Time of Time for BFS: Mon Jan  8 15:48:28 2024

TEPS for BFS 3563 is 2.20318e+08
Running BFS 3564
Current Time of Running BFS: Mon Jan  8 15:48:28 2024

Time for BFS 3564 is 0.073757
Current Time of Time for BFS: Mon Jan  8 15:48:28 2024

TEPS for BFS 3564 is 2.27446e+08
Running BFS 3565
Current Time of Running BFS: Mon Jan  8 15:48:28 2024

Time for BFS 3565 is 0.072463
Current Time of Time for BFS: Mon Jan  8 15:48:28 2024

TEPS for BFS 3565 is 2.3151e+08
Running BFS 3566
Current Time of Running BFS: Mon Jan  8 15:48:29 2024

Time for BFS 3566 is 0.074659
Current Time of Time for BFS: Mon Jan  8 15:48:29 2024

TEPS for BFS 3566 is 2.24699e+08
Running BFS 3567
Current Time of Running BFS: Mon Jan  8 15:48:29 2024

Time for BFS 3567 is 0.080107
Current Time of Time for BFS: Mon Jan  8 15:48:29 2024

TEPS for BFS 3567 is 2.09418e+08
Running BFS 3568
Current Time of Running BFS: Mon Jan  8 15:48:29 2024

Time for BFS 3568 is 0.077384
Current Time of Time for BFS: Mon Jan  8 15:48:29 2024

TEPS for BFS 3568 is 2.16785e+08
Running BFS 3569
Current Time of Running BFS: Mon Jan  8 15:48:29 2024

Time for BFS 3569 is 0.077321
Current Time of Time for BFS: Mon Jan  8 15:48:29 2024

TEPS for BFS 3569 is 2.16964e+08
Running BFS 3570
Current Time of Running BFS: Mon Jan  8 15:48:29 2024

Time for BFS 3570 is 0.073712
Current Time of Time for BFS: Mon Jan  8 15:48:29 2024

TEPS for BFS 3570 is 2.27586e+08
Running BFS 3571
Current Time of Running BFS: Mon Jan  8 15:48:29 2024

Time for BFS 3571 is 0.074922
Current Time of Time for BFS: Mon Jan  8 15:48:29 2024

TEPS for BFS 3571 is 2.23909e+08
Running BFS 3572
Current Time of Running BFS: Mon Jan  8 15:48:29 2024

Time for BFS 3572 is 0.075831
Current Time of Time for BFS: Mon Jan  8 15:48:29 2024

TEPS for BFS 3572 is 2.21228e+08
Running BFS 3573
Current Time of Running BFS: Mon Jan  8 15:48:29 2024

Time for BFS 3573 is 0.070372
Current Time of Time for BFS: Mon Jan  8 15:48:29 2024

TEPS for BFS 3573 is 2.38387e+08
Running BFS 3574
Current Time of Running BFS: Mon Jan  8 15:48:29 2024

Time for BFS 3574 is 0.076196
Current Time of Time for BFS: Mon Jan  8 15:48:29 2024

TEPS for BFS 3574 is 2.20168e+08
Running BFS 3575
Current Time of Running BFS: Mon Jan  8 15:48:29 2024

Time for BFS 3575 is 0.076192
Current Time of Time for BFS: Mon Jan  8 15:48:29 2024

TEPS for BFS 3575 is 2.20177e+08
Running BFS 3576
Current Time of Running BFS: Mon Jan  8 15:48:29 2024

Time for BFS 3576 is 0.075657
Current Time of Time for BFS: Mon Jan  8 15:48:29 2024

TEPS for BFS 3576 is 2.21734e+08
Running BFS 3577
Current Time of Running BFS: Mon Jan  8 15:48:29 2024

Time for BFS 3577 is 0.075265
Current Time of Time for BFS: Mon Jan  8 15:48:30 2024

TEPS for BFS 3577 is 2.22891e+08
Running BFS 3578
Current Time of Running BFS: Mon Jan  8 15:48:30 2024

Time for BFS 3578 is 0.078641
Current Time of Time for BFS: Mon Jan  8 15:48:30 2024

TEPS for BFS 3578 is 2.1332e+08
Running BFS 3579
Current Time of Running BFS: Mon Jan  8 15:48:30 2024

Time for BFS 3579 is 0.075800
Current Time of Time for BFS: Mon Jan  8 15:48:30 2024

TEPS for BFS 3579 is 2.21318e+08
Running BFS 3580
Current Time of Running BFS: Mon Jan  8 15:48:30 2024

Time for BFS 3580 is 0.077414
Current Time of Time for BFS: Mon Jan  8 15:48:30 2024

TEPS for BFS 3580 is 2.16702e+08
Running BFS 3581
Current Time of Running BFS: Mon Jan  8 15:48:30 2024

Time for BFS 3581 is 0.076387
Current Time of Time for BFS: Mon Jan  8 15:48:30 2024

TEPS for BFS 3581 is 2.19616e+08
Running BFS 3582
Current Time of Running BFS: Mon Jan  8 15:48:30 2024

Time for BFS 3582 is 0.076626
Current Time of Time for BFS: Mon Jan  8 15:48:30 2024

TEPS for BFS 3582 is 2.1893e+08
Running BFS 3583
Current Time of Running BFS: Mon Jan  8 15:48:30 2024

Time for BFS 3583 is 0.076689
Current Time of Time for BFS: Mon Jan  8 15:48:30 2024

TEPS for BFS 3583 is 2.18751e+08
Running BFS 3584
Current Time of Running BFS: Mon Jan  8 15:48:30 2024

Time for BFS 3584 is 0.074890
Current Time of Time for BFS: Mon Jan  8 15:48:30 2024

TEPS for BFS 3584 is 2.24005e+08
Running BFS 3585
Current Time of Running BFS: Mon Jan  8 15:48:30 2024

Time for BFS 3585 is 0.073164
Current Time of Time for BFS: Mon Jan  8 15:48:30 2024

TEPS for BFS 3585 is 2.2929e+08
Running BFS 3586
Current Time of Running BFS: Mon Jan  8 15:48:30 2024

Time for BFS 3586 is 0.075401
Current Time of Time for BFS: Mon Jan  8 15:48:30 2024

TEPS for BFS 3586 is 2.22489e+08
Running BFS 3587
Current Time of Running BFS: Mon Jan  8 15:48:30 2024

Time for BFS 3587 is 0.079191
Current Time of Time for BFS: Mon Jan  8 15:48:30 2024

TEPS for BFS 3587 is 2.11841e+08
Running BFS 3588
Current Time of Running BFS: Mon Jan  8 15:48:30 2024

Time for BFS 3588 is 0.075863
Current Time of Time for BFS: Mon Jan  8 15:48:31 2024

TEPS for BFS 3588 is 2.21132e+08
Running BFS 3589
Current Time of Running BFS: Mon Jan  8 15:48:31 2024

Time for BFS 3589 is 0.076165
Current Time of Time for BFS: Mon Jan  8 15:48:31 2024

TEPS for BFS 3589 is 2.20258e+08
Running BFS 3590
Current Time of Running BFS: Mon Jan  8 15:48:31 2024

Time for BFS 3590 is 0.077551
Current Time of Time for BFS: Mon Jan  8 15:48:31 2024

TEPS for BFS 3590 is 2.16318e+08
Running BFS 3591
Current Time of Running BFS: Mon Jan  8 15:48:31 2024

Time for BFS 3591 is 0.076789
Current Time of Time for BFS: Mon Jan  8 15:48:31 2024

TEPS for BFS 3591 is 2.18466e+08
Running BFS 3592
Current Time of Running BFS: Mon Jan  8 15:48:31 2024

Time for BFS 3592 is 0.075411
Current Time of Time for BFS: Mon Jan  8 15:48:31 2024

TEPS for BFS 3592 is 2.22459e+08
Running BFS 3593
Current Time of Running BFS: Mon Jan  8 15:48:31 2024

Time for BFS 3593 is 0.072644
Current Time of Time for BFS: Mon Jan  8 15:48:31 2024

TEPS for BFS 3593 is 2.30932e+08
Running BFS 3594
Current Time of Running BFS: Mon Jan  8 15:48:31 2024

Time for BFS 3594 is 0.077118
Current Time of Time for BFS: Mon Jan  8 15:48:31 2024

TEPS for BFS 3594 is 2.17534e+08
Running BFS 3595
Current Time of Running BFS: Mon Jan  8 15:48:31 2024

Time for BFS 3595 is 0.073166
Current Time of Time for BFS: Mon Jan  8 15:48:31 2024

TEPS for BFS 3595 is 2.29283e+08
Running BFS 3596
Current Time of Running BFS: Mon Jan  8 15:48:31 2024

Time for BFS 3596 is 0.076305
Current Time of Time for BFS: Mon Jan  8 15:48:31 2024

TEPS for BFS 3596 is 2.19853e+08
Running BFS 3597
Current Time of Running BFS: Mon Jan  8 15:48:31 2024

Time for BFS 3597 is 0.100489
Current Time of Time for BFS: Mon Jan  8 15:48:31 2024

TEPS for BFS 3597 is 1.66941e+08
Running BFS 3598
Current Time of Running BFS: Mon Jan  8 15:48:31 2024

Time for BFS 3598 is 0.095723
Current Time of Time for BFS: Mon Jan  8 15:48:31 2024

TEPS for BFS 3598 is 1.75254e+08
Running BFS 3599
Current Time of Running BFS: Mon Jan  8 15:48:31 2024

Time for BFS 3599 is 0.116033
Current Time of Time for BFS: Mon Jan  8 15:48:32 2024

TEPS for BFS 3599 is 1.44578e+08
Running BFS 3600
Current Time of Running BFS: Mon Jan  8 15:48:32 2024

Time for BFS 3600 is 0.077749
Current Time of Time for BFS: Mon Jan  8 15:48:32 2024

TEPS for BFS 3600 is 2.15768e+08
Running BFS 3601
Current Time of Running BFS: Mon Jan  8 15:48:32 2024

Time for BFS 3601 is 0.078837
Current Time of Time for BFS: Mon Jan  8 15:48:32 2024

TEPS for BFS 3601 is 2.12792e+08
Running BFS 3602
Current Time of Running BFS: Mon Jan  8 15:48:32 2024

Time for BFS 3602 is 0.077917
Current Time of Time for BFS: Mon Jan  8 15:48:32 2024

TEPS for BFS 3602 is 2.15305e+08
Running BFS 3603
Current Time of Running BFS: Mon Jan  8 15:48:32 2024

Time for BFS 3603 is 0.074435
Current Time of Time for BFS: Mon Jan  8 15:48:32 2024

TEPS for BFS 3603 is 2.25376e+08
Running BFS 3604
Current Time of Running BFS: Mon Jan  8 15:48:32 2024

Time for BFS 3604 is 0.075262
Current Time of Time for BFS: Mon Jan  8 15:48:32 2024

TEPS for BFS 3604 is 2.22899e+08
Running BFS 3605
Current Time of Running BFS: Mon Jan  8 15:48:32 2024

Time for BFS 3605 is 0.077473
Current Time of Time for BFS: Mon Jan  8 15:48:32 2024

TEPS for BFS 3605 is 2.16538e+08
Running BFS 3606
Current Time of Running BFS: Mon Jan  8 15:48:32 2024

Time for BFS 3606 is 0.079460
Current Time of Time for BFS: Mon Jan  8 15:48:32 2024

TEPS for BFS 3606 is 2.11124e+08
Running BFS 3607
Current Time of Running BFS: Mon Jan  8 15:48:32 2024

Time for BFS 3607 is 0.076857
Current Time of Time for BFS: Mon Jan  8 15:48:32 2024


"""

# Extract the times for BFS using regex
time_pattern = re.compile(r"Time for BFS \d+ is ([\d.]+)")
times = [float(match) for match in time_pattern.findall(bfs_text)]

# Calculate the median
times.sort()
n = len(times)
median = times[n // 2] if n % 2 != 0 else (times[n // 2 - 1] + times[n // 2]) / 2

print(median)

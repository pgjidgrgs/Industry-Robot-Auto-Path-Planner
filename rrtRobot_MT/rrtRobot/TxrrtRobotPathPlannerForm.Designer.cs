using System;
using System.Windows.Forms.VisualStyles;
using Tecnomatix.Engineering;
using Tecnomatix.Engineering.Ui;

namespace rrtRobot
{
    partial class TxrrtRobotPathPlannerForm
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        public void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(TxrrtRobotPathPlannerForm));
            this.m_txTCPFrame = new System.Windows.Forms.TextBox();
            this.m_txRobotName = new Tecnomatix.Engineering.Ui.TxObjEditBoxCtrl();
            this.TCP = new System.Windows.Forms.Label();
            this.Robot_Name = new System.Windows.Forms.Label();
            this.Collision_Src = new System.Windows.Forms.Label();
            this.Group_Collision = new System.Windows.Forms.GroupBox();
            this.m_collisionListPick = new Tecnomatix.Engineering.Ui.TxObjComboBoxCtrl();
            this.label2 = new System.Windows.Forms.Label();
            this.m_pathGenerate = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.PathprogressBar = new System.Windows.Forms.ProgressBar();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.label3 = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.label1 = new System.Windows.Forms.Label();
            this.m_spotDirec = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.Group_Collision.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.SuspendLayout();
            // 
            // m_txTCPFrame
            // 
            resources.ApplyResources(this.m_txTCPFrame, "m_txTCPFrame");
            this.m_txTCPFrame.Name = "m_txTCPFrame";
            this.m_txTCPFrame.ReadOnly = true;
            // 
            // m_txRobotName
            // 
            resources.ApplyResources(this.m_txRobotName, "m_txRobotName");
            this.m_txRobotName.KeepFaceEmphasizedWhenControlIsNotFocused = true;
            this.m_txRobotName.ListenToPick = true;
            this.m_txRobotName.Name = "m_txRobotName";
            this.m_txRobotName.Object = null;
            this.m_txRobotName.PickAndClear = false;
            this.m_txRobotName.PickLevel = Tecnomatix.Engineering.Ui.TxPickLevel.Component;
            this.m_txRobotName.PickOnly = false;
            this.m_txRobotName.ReadOnly = false;
            this.m_txRobotName.ValidatorType = Tecnomatix.Engineering.Ui.TxValidatorType.Robot;
            this.m_txRobotName.Picked += new Tecnomatix.Engineering.Ui.TxObjEditBoxCtrl_PickedEventHandler(this.m_txRobotName_Picked);
            // 
            // TCP
            // 
            resources.ApplyResources(this.TCP, "TCP");
            this.TCP.Name = "TCP";
            // 
            // Robot_Name
            // 
            resources.ApplyResources(this.Robot_Name, "Robot_Name");
            this.Robot_Name.Name = "Robot_Name";
            // 
            // Collision_Src
            // 
            resources.ApplyResources(this.Collision_Src, "Collision_Src");
            this.Collision_Src.Name = "Collision_Src";
            // 
            // Group_Collision
            // 
            this.Group_Collision.Controls.Add(this.label4);
            this.Group_Collision.Controls.Add(this.m_collisionListPick);
            this.Group_Collision.Controls.Add(this.label2);
            this.Group_Collision.Controls.Add(this.Collision_Src);
            resources.ApplyResources(this.Group_Collision, "Group_Collision");
            this.Group_Collision.Name = "Group_Collision";
            this.Group_Collision.TabStop = false;
            // 
            // m_collisionListPick
            // 
            this.m_collisionListPick.DropDownHeight = 106;
            this.m_collisionListPick.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDown;
            resources.ApplyResources(this.m_collisionListPick, "m_collisionListPick");
            this.m_collisionListPick.ListenToPick = true;
            this.m_collisionListPick.Name = "m_collisionListPick";
            this.m_collisionListPick.Object = null;
            this.m_collisionListPick.PickLevel = Tecnomatix.Engineering.Ui.TxPickLevel.Component;
            this.m_collisionListPick.ValidatorType = Tecnomatix.Engineering.Ui.TxValidatorType.AnyLocatableObject;
            this.m_collisionListPick.Picked += new Tecnomatix.Engineering.Ui.TxObjComboBoxCtrl_PickedEventHandler(this.m_collisionListPicked);
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            // 
            // m_pathGenerate
            // 
            resources.ApplyResources(this.m_pathGenerate, "m_pathGenerate");
            this.m_pathGenerate.Name = "m_pathGenerate";
            this.m_pathGenerate.UseVisualStyleBackColor = true;
            this.m_pathGenerate.Click += new System.EventHandler(this.m_pathGenerate_Click);
            // 
            // button1
            // 
            resources.ApplyResources(this.button1, "button1");
            this.button1.Name = "button1";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // PathprogressBar
            // 
            resources.ApplyResources(this.PathprogressBar, "PathprogressBar");
            this.PathprogressBar.Name = "PathprogressBar";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.PathprogressBar);
            resources.ApplyResources(this.groupBox1, "groupBox1");
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.TabStop = false;
            // 
            // label3
            // 
            resources.ApplyResources(this.label3, "label3");
            this.label3.Name = "label3";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.label1);
            this.groupBox2.Controls.Add(this.m_txTCPFrame);
            this.groupBox2.Controls.Add(this.m_txRobotName);
            this.groupBox2.Controls.Add(this.TCP);
            this.groupBox2.Controls.Add(this.Robot_Name);
            resources.ApplyResources(this.groupBox2, "groupBox2");
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.TabStop = false;
            // 
            // label1
            // 
            resources.ApplyResources(this.label1, "label1");
            this.label1.Name = "label1";
            // 
            // m_spotDirec
            // 
            resources.ApplyResources(this.m_spotDirec, "m_spotDirec");
            this.m_spotDirec.Name = "m_spotDirec";
            // 
            // label4
            // 
            resources.ApplyResources(this.label4, "label4");
            this.label4.Name = "label4";
            // 
            // TxrrtRobotPathPlannerForm
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.m_pathGenerate);
            this.Controls.Add(this.Group_Collision);
            this.Controls.Add(this.groupBox1);
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.Name = "TxrrtRobotPathPlannerForm";
            this.SemiModal = false;
            this.ShouldCloseOnDocumentUnloading = true;
            this.Group_Collision.ResumeLayout(false);
            this.Group_Collision.PerformLayout();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

       

        #endregion
        private System.Windows.Forms.TextBox m_txTCPFrame;
        private Tecnomatix.Engineering.Ui.TxObjEditBoxCtrl m_txRobotName;
        private System.Windows.Forms.Label TCP;
        private System.Windows.Forms.Label Robot_Name;
       
        private System.Windows.Forms.Label Collision_Src;
        private System.Windows.Forms.GroupBox Group_Collision;
        private System.Windows.Forms.Button m_pathGenerate;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.ProgressBar PathprogressBar;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private TxObjComboBoxCtrl m_collisionListPick;
        private System.Windows.Forms.Label label1;
        
        public System.Windows.Forms.Label m_spotDirec;
        private System.Windows.Forms.Label label4;
    }
}